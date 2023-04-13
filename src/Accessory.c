#include "libusb.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"

#include <stdio.h>
#include <pthread.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>


#define Debug(_fmt, _args...)      {printf("\033[40;35m###%s line %d ",__func__, __LINE__);printf(_fmt, ##_args); printf("\033[0m");}
#define IOBufferMaxSize 32768

int exit_flag = 0;

typedef struct usb_transfer{
    int stop_transfer;
    int usbActive;

    pthread_t pid;
    pthread_mutex_t mutex;
    pthread_cond_t cond;

    struct libusb_transfer *xfr;
}usb_transfer;

typedef struct usb_device{
    uint16_t pid;                       /*product id*/
    uint16_t vid;                       /*vendor id*/
    int event;                          /*hotplug event标志位*/         
    uint8_t InEndpointAddress;          /*input 端点的地址*/
    uint8_t OutEndpointAddress;         /*out 端点的地址*/
    uint8_t InterfaceNumber;            /*接口下标号*/
    int ConfigureNum;                   /*可配置数量*/
    int exit_flag;                      /*退出标志位*/
    uint16_t max_input_size;
    uint16_t max_output_size;
    uint8_t *io_buffer;
    unsigned int len;

    usb_transfer *usb_transfer;

    pthread_t event_thread_id;
    pthread_mutex_t mutex;

    pthread_t encodec_thread;
    pthread_cond_t read_cond;
    pthread_mutex_t read_mutex;
    
    libusb_context *cxt;
    libusb_device *dev;
    libusb_device_handle *handle;
    libusb_hotplug_callback_handle hotplug_handle;
}usb_device;


char *AndroidCheckInformation[6] = {
    "NEWLINK, Inc.",                                                      /*设备厂商*/
    "NewlinkAndroidCableCast",                                           /*模型名称*/
    "Android CableCast",                                                 /*相关描述信息*/
    "1.0",                                                               /*版本*/
    "http://www.newlinksz.com/zyftp/ZYUpdate/NewLinkApp/Accessory.apk",  /*URI*/
    "1234567890"                                                         /*序列号*/
};

enum accessory_flag{
    VID_GOOGLE = 0x18D1,

    PID_AOA_ACC = 0x2D00,
    PID_AOA_ACC_ADB = 0x2D01,
    PID_AOA_AU = 0x2D02,
    PID_AOA_AU_ADB = 0x2D03,
    PID_AOA_ACC_AU = 0x2D04,
    PID_AOA_ACC_AU_ADB = 0x2D05,
}typedef accessory_flag;

/**
 * @brief 
 * hotplug响应处理函数,把插入和拔出事件赋值给dev中的event,同时把产生事件的dev相关信息赋值给 usb_device
 * @param ctx 
 * @param dev      发生事件的dev
 * @param event    发生的事件
 * @param user_data 传入的自定义数据
 * @return int 
 * success return 0
 * fail return -1
 */
int LIBUSB_CALL usb_hotplug_callback(libusb_context *ctx, libusb_device *dev, libusb_hotplug_event event, void *user_data)
{
    struct libusb_device_descriptor desc;
    int ret;
    usb_device *p = (usb_device *)user_data;

    pthread_mutex_lock(&p->mutex);

    if((ret = libusb_get_device_descriptor(dev,&desc))!= LIBUSB_SUCCESS)
    {
        Debug("%s %d err is %s\n",__func__,__LINE__,libusb_error_name(ret));
        return -1;
    }
    
    if(event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED){ /*插入事件*/
        if(((desc.idVendor == 0x0bda) && (desc.idProduct == 0x8731)) /*过滤 usb wifi设备*/
        || ((desc.idVendor == 0x1d6b) && (desc.idProduct == 0x0002 || desc.idProduct == 0x0003)))/*过滤 usb root hub*/
        {
            Debug("get usb root hub device\n");
            pthread_mutex_unlock(&p->mutex);
            return 0;
        }

        Debug("usb device arrived %04x : %04x \n",desc.idVendor,desc.idProduct);
        
        /*获取插入设备的相关信息*/
        p->vid = desc.idVendor;
        p->pid = desc.idProduct;
        p->event = event;
        p->ConfigureNum = desc.bNumConfigurations;
        p->dev = dev;
        p->InEndpointAddress = 0;
        p->OutEndpointAddress = 0;    
    }
    else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT){/*拔出事件*/
       Debug("usb device left  %04x : %04x\n",desc.idVendor,desc.idProduct);
       p->event = event;
    }
    else{
        Debug("usb device event warning !\n");
        p->event = event;
    }

    pthread_mutex_unlock(&p->mutex);
    return 0;
}

/**
 * @brief 
 * 向 Android设备发送数据,传输模式为控制传输
 * @param handle 擦偶哦句柄
 * @param data 传输的数据
 * @param req 请求报文
 * @param index 索引信息
 * @param timeout 超时时间
 * @return int 
 * success return 1
 * fila return -1
 */
 int usb_send_ctrl(libusb_device_handle *handle,char * data,int req,int index,int timeout)
 {
    if(NULL == handle)
        return -1;
    int ret;
    if(NULL == data){
        ret = libusb_control_transfer(handle,0x40,req,0,index,NULL,0,timeout);
    }
    else{
        ret = libusb_control_transfer(handle,0x40,req,0,index,data,strlen(data) + 1,timeout);
    }

    if(ret < 0)
        return ret;

    return 1;
 }

/**
 * @brief Set the device to accessory object
 * 尝试把usb device设备切换成 Accessory模式
 * @param dev 需要切换模式的dev
 * @return int 
 * success return 1
 * fail return -1
 */
int set_device_to_accessory(libusb_device *dev)
{
    int ret;
    int aoa_version;
    uint8_t IOBuffer[2] = {0};
    libusb_device_handle *handle = NULL;
    /*打开usb设备*/
    if((ret = libusb_open(dev,&handle))!= 0){
        Debug("%s %d open usb device err %s\n",__func__,__LINE__,libusb_error_name(ret));
        return -1;
    }

    /*如果usb device已经连接了kernel driver,则进行detach*/
    if(libusb_kernel_driver_active(handle,0) > 0){
        Debug("usb device is register kernel driver\n");
        if((ret = libusb_detach_kernel_driver(handle,0))!= LIBUSB_SUCCESS)
            goto EXIT;
    }

    /*发送51报文 查询设备是否支持accessory模式,并获取设备的accessory的版本号 目前只有1.0和2.0两个版本*/
    if((ret = libusb_control_transfer(handle,
    0xC0, /*usb请求类型*/
    51,/*51报文*/
    0,/*命令信息*/
    0,/*索引信息*/
    IOBuffer,/*发送/接收 data*/
    2,/*发送/接收的 size 大小*/
    2000/*超时time*/
    )) < 0){
        Debug("%s %d usb control transfer data err %s\n",__func__,__LINE__,libusb_error_name(ret));
        goto EXIT;
    }

    aoa_version = IOBuffer[1] << 8 | IOBuffer[0];
    Debug("Android device Accessory Version is %d\n",aoa_version);
    /*判断AOA协议版本 如果返回值为非0,则表示设备支持accessory*/
    if(aoa_version!= 1 && aoa_version!= 2){
        Debug("usb device is unsupport Accessory\n");
        goto EXIT;
    }
    
    usleep(1000);
    /*发送52报文给 android设备发送验证字符串信息,android设备接收到信息后,会根据前三个字符信息
    试图找到与配件匹配的apk,如果没找到则会根据URI提示用户安装apk*/
    for(int i = 0;i < 6;i++){
        if((ret = usb_send_ctrl(handle,AndroidCheckInformation[i],52,i,2000))!= 1){
            Debug("usb control transfer err %s\n",libusb_error_name(ret));
            goto EXIT;   
        }                
    }

    /*发送53报文尝试启动Android设备的 accessory模式*/
    if((ret = usb_send_ctrl(handle,NULL,53,0,2000))!= 1){
        Debug("usb control transfer err %s\n",libusb_error_name(ret));
        goto EXIT;   
    }

    Debug("change Android device to Accessory success\n");
    libusb_close(handle);
    return 1;

    EXIT:
        libusb_close(handle);
        return -1;

}

/**
 * @brief 
 * 通过 vendor_id 和 product_id检查dev是否处于 Accessory模式
 * @param id_vendor 
 * @param id_product 
 * @return int 
 * 处于Accessory return 1
 * 其他 return 0
 */
int check_device_accessory(uint16_t id_vendor,uint16_t id_product)
{
    if(id_vendor == VID_GOOGLE){
        switch (id_product)
        {
        case PID_AOA_ACC:
        case PID_AOA_ACC_ADB:
        case PID_AOA_ACC_AU:
        case PID_AOA_ACC_AU_ADB:
            return 1;
        
        /*音频*/
        case PID_AOA_AU:
        case PID_AOA_AU_ADB:
            break;

        default:
            break;
        }
    }
    return 0;
}

/**
 * @brief Get the usb endpoint object
 * 获取处于accessory模式下Android设备的输入和输出端点
 * @param desc 接口描述信息
 * @param dev usb_device
 * @return int 
 * success return 1
 * fail return libusb错误码
 */
int get_usb_endpoint(struct libusb_interface_descriptor *desc,usb_device *dev)
{   
    if(NULL == desc)
        return -1;
    int ret,i;
    struct libusb_endpoint_descriptor *ept = NULL;
    
    for(i = 0; i < desc->bNumEndpoints; i++){
        ept = (struct libusb_endpoint_descriptor *)(&desc->endpoint[i]);
        if(ept->bmAttributes == LIBUSB_TRANSFER_TYPE_BULK){
            if(ept->bEndpointAddress & LIBUSB_ENDPOINT_IN){
                Debug("get Input Endpoint Address %02x\n",ept->bEndpointAddress);
                dev->InEndpointAddress = ept->bEndpointAddress;
                dev->max_input_size = ept->wMaxPacketSize;
            }
            else if(ept->bEndpointAddress & LIBUSB_ENDPOINT_OUT){
                dev->OutEndpointAddress = ept->bEndpointAddress;
                dev->max_output_size = ept->wMaxPacketSize;
                Debug("get Out Endpoint Address %02x\n",dev->OutEndpointAddress);
            }
        }

        if(dev->InEndpointAddress && dev->OutEndpointAddress){
            ret = 1;
            break;
        }
    }
    return ret;
}


/**
 * @brief Get the usb comm endpoint object
 * 获取Android设备使用的接口配置,和输入输出端点
 * @param dev 
 * @return int
 * success return 1
 * fail return -1 
 */
int get_usb_comm_endpoint(usb_device *dev)
{
    int ret,i,j,k;
    struct libusb_config_descriptor *confg = NULL;
    struct libusb_interface_descriptor *inter_desc = NULL;
    struct libusb_interface *inter = NULL;

    for(i = 0; i < dev->ConfigureNum; i++){
       if(NULL != dev->dev){
            if((ret = libusb_get_config_descriptor(dev->dev,0,&confg))!= LIBUSB_SUCCESS){
                Debug("%s %d get usb device config_descriptor err %s\n",__func__,__LINE__,libusb_error_name(ret));
                continue;
            }
            
            for(j = 0; j < confg->bNumInterfaces; j++){
                inter = (struct libusb_interface *)(&confg->interface[j]);
                
                for(k = 0; k < inter->num_altsetting; k++){
                    inter_desc = (struct libusb_interface_descriptor *)(&inter->altsetting[k]);

                    if(inter_desc->bInterfaceClass == LIBUSB_CLASS_VENDOR_SPEC &&
                    inter_desc->bInterfaceSubClass == LIBUSB_CLASS_VENDOR_SPEC){
                        if(dev->InEndpointAddress <= 0 && dev->OutEndpointAddress <=0){
                            if(get_usb_endpoint(inter_desc,dev)){
                                    dev->InterfaceNumber = inter_desc->bInterfaceNumber;
                                    ret = 1;
                                    break;
                            }
                        }
                    }

                }
            }

            libusb_free_config_descriptor(confg);
            if(ret)
                break; 
       } 
    }

    return ret;
}


/**
 * @brief 
 * 打开ubs设备,并声明所使用的interface
 * @param dev 
 * usb_device
 * @return int 
 * success return 1
 * fail return-1
 */
int open_usb_device(usb_device *dev)
{
    int ret;
    if((ret = libusb_open(dev->dev,&dev->handle))!= LIBUSB_SUCCESS){
        Debug("%s  %dopen usb device err %s\n",__func__,__LINE__,libusb_error_name(ret));
        return -1;
    }

    if((ret = libusb_claim_interface(dev->handle,dev->InterfaceNumber))!= LIBUSB_SUCCESS){
        Debug("%s  %dusb claim interface err %s\n",__func__,__LINE__,libusb_error_name(ret));
        libusb_close(dev->handle);
        dev->handle = NULL;
        return -1;
    }

    return 1;
}

int init_usb_transfer(usb_transfer *p)
{
    if(NULL == p)
        return -1;
    p->xfr = libusb_alloc_transfer(0);

    if(NULL == p->xfr)
        return -1;
    
    p->stop_transfer = 0;
    p->usbActive = 0;
    pthread_mutex_init(&p->mutex,NULL);
    pthread_cond_init(&p->cond,NULL);
    return 0;
}


void usb_rx_cb(struct libusb_transfer *t)
{
    usb_transfer *p = (usb_transfer *)t->user_data;

    pthread_mutex_lock(&p->mutex);

    if(p->usbActive){
        p->usbActive = 0;
        pthread_cond_signal(&p->cond);
    }

    pthread_mutex_unlock(&p->mutex);
}


void *usb_transfer_thread(void *arg)
{
    int ret;
    int rx_byts;
    int tx_byts;
    long send_size;
    usb_device *dev = (usb_device *)arg;
    uint8_t Buffer[dev->max_input_size];

    dev->io_buffer = Buffer;
    FILE *f = fopen("frame.h264","a+");
    if(NULL == f)
        return NULL;

    libusb_fill_bulk_transfer(dev->usb_transfer->xfr,dev->handle,dev->InEndpointAddress,
    Buffer,sizeof(Buffer),usb_rx_cb,(void *)dev->usb_transfer,1000);

    unsigned char uchfirst = 1;
    unsigned char *uchdata = NULL;
    unsigned char uchcomplete = 0;
    unsigned char uchgetlen = 0;
    unsigned char uchoffset = 0;
    unsigned int uipos = 0;
    unsigned int uilen = 0;
    uchdata = (unsigned char *)malloc(1024 *1024);

    while (!dev->usb_transfer->stop_transfer){
        pthread_mutex_lock(&dev->usb_transfer->mutex);
        dev->usb_transfer->usbActive = 1;
        ret = libusb_submit_transfer(dev->usb_transfer->xfr);

        if(ret!= LIBUSB_SUCCESS){
            Debug("usb submit transfer %s\n",libusb_error_name(ret));
            dev->usb_transfer->stop_transfer = 1;
            dev->usb_transfer->usbActive = 0;
            pthread_mutex_unlock(&dev->usb_transfer->mutex);
            break;
        }
        pthread_cond_wait(&dev->usb_transfer->cond,&dev->usb_transfer->mutex);

        if(dev->usb_transfer->usbActive)
            Debug("wait, unlock but usbActive!\n");

        pthread_mutex_unlock(&dev->usb_transfer->mutex);
        if(dev->usb_transfer->stop_transfer)
            break;
        
        switch (dev->usb_transfer->xfr->status){
        case LIBUSB_TRANSFER_COMPLETED:
#if 0
            rx_byts = dev->usb_transfer->xfr->actual_length;
            send_size = 0;
            tx_byts = 0;
            while(send_size < rx_byts && !dev->usb_transfer->stop_transfer){
                Debug("usb send ------> %d:\n %s\n",rx_byts,Buffer);
                send_size += rx_byts;
            }
#endif
            rx_byts = dev->usb_transfer->xfr->actual_length;
            if(!uchcomplete){
                if((uipos+rx_byts) > 1024*1024)
                {
                    Debug("video buf size out of bounds[%d,%d]\n",uipos,rx_byts);
                    break;
                }
                memcpy(uchdata+uipos,Buffer,rx_byts);
                uipos = uipos + rx_byts; //uipos 记录上次write的最后位置
                if(!uchoffset)
                {
                    unsigned char *uchbuf = (unsigned char *)&uilen;
                    //memcpy(&uilen,uchdata,4);
                    uchbuf[0] = uchdata[3];
                    uchbuf[1] = uchdata[2];
                    uchbuf[2] = uchdata[1];
                    uchbuf[3] = uchdata[0];
                    uchoffset = 4;
                    //Debug("=========%s_%d_%d\n",__FUNCTION__,__LINE__,uilen);
                }

                if((uipos-uchoffset) > uilen)
                    uchcomplete = 1;
            }

            if(uchcomplete){
                if(uchfirst){
                    uchcomplete = 0;
                    fwrite((const void *)(uchdata+uchoffset),1,uilen,f);
                    uipos = uipos - uilen - uchoffset;
                    if(uipos > 0)
                        memcpy(uchdata,uchdata+uilen+uchoffset,uipos);
                    uchoffset = 0;
                    uchcomplete = 0;
                    uchfirst = 0;
                }
                else{
                    uchcomplete = 0;
                    fwrite((const void *)(uchdata+uchoffset),1,uilen,f);
                    uipos = uipos - uilen - uchoffset;
                    if(uipos > 0)
                        memcpy(uchdata,uchdata+uilen+uchoffset,uipos);
                    uchoffset = 0;
                    uchcomplete = 0;
                }
            }
            dev->len = uilen;
            pthread_cond_signal(&dev->read_cond);
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            fclose(f);
            Debug("usb transfer is cancealled\n");
            dev->usb_transfer->stop_transfer = 1;
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            fclose(f);
            Debug("Android device is left\n");
            dev->usb_transfer->stop_transfer = 1;
            break;
        default:
            break;
        }
    }

    dev->usb_transfer->stop_transfer = 1;
    pthread_exit(0);
    return NULL;
}

int start_usb_transfer(usb_device *dev)
{
    int ret;
    if(dev->usb_transfer == NULL){
        dev->usb_transfer = (usb_transfer *)malloc(sizeof(usb_transfer));
        if(dev->usb_transfer == NULL)
            return -1;
    }

    if(init_usb_transfer(dev->usb_transfer)!= 0){
        Debug("%s  %d init usb device transfererr\n",__func__,__LINE__);
        free(dev->usb_transfer);
        return -1;
    }

    ret = pthread_create(&dev->usb_transfer->pid,NULL,usb_transfer_thread,(void *)dev);
    if(ret < 0){
        Debug("%s  %d create usb device thread err\n",__func__,__LINE__);
        free(dev->usb_transfer);
        return -1;
    }
        
    return 0;
}

/**
 * @brief 
 * usb 插入事件处理,将设备切换为AOA模式,并打开usb设备
 * @param dev 
 * @return int 
 * success return 1
 * fail return -1
 */
int usb_hotplug_arrived(usb_device *dev)
{
    int ret;
    pthread_mutex_lock(&dev->mutex);
    /*判断USB设备是否处于 AOA模式*/
    if((ret = check_device_accessory(dev->vid,dev->pid))!= 1){
        /*尝试开启配件模式*/
        if((ret = set_device_to_accessory(dev->dev))!= 1){
            Debug("can not change the usb device to accessory\n");
            goto ERR;
        }
    }else{ /*已经处于AOA模式 需要获取usb transfer 和 recv 的端点*/
        if((ret = get_usb_comm_endpoint(dev))!= 1){
            Debug("%s  %dget usb device transfer endpoint err\n",__func__,__LINE__);
            goto ERR;
        }
        /*打开usb设备 声明使用的接口*/
        if((ret = open_usb_device(dev))!= 1){
            Debug("%s  %dget usb device transfer endpoint err\n",__func__,__LINE__);
            goto ERR;
        }
        /*开启usb通信 线程*/
        if((ret = start_usb_transfer(dev))!= 0){
            Debug("%s  %d create usb device transfer thread err\n",__func__,__LINE__);
            goto ERR;
        }
    }

    Debug("USB Device Set Accessory Success\n");
    pthread_mutex_unlock(&dev->mutex);
    return 1;

    ERR:
        pthread_mutex_unlock(&dev->mutex);
        return -1;
}

/**
 * @brief 
 * usb 拔出事件处理
 * @param dev 
 */
void usb_hotplug_left(usb_device *dev)
{
    if(NULL == dev)
        return;
    
    if(dev->handle){
        libusb_cancel_transfer(dev->usb_transfer->xfr);
        pthread_join(dev->usb_transfer->pid,NULL);
        Debug("usb transfer thread is exit!\n");
        libusb_free_transfer(dev->usb_transfer->xfr);
        libusb_release_interface(dev->handle,dev->InterfaceNumber);
        libusb_close(dev->handle);
        dev->handle = NULL;
        dev->exit_flag = 1;
        exit_flag = 1;
        Debug("usb device is left success\n");
    }
    
    return;
}

/**
 * @brief 
 * us热插拔事件响应
 * @param arg 
 * @return void* 
 */
void *usb_event_monitor_process(void *arg)
{
    usb_device *dev = (usb_device *)arg;
    int ret;
    if(NULL == dev)
        return NULL;
    while (1)
    {
        switch (dev->event){
            case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
                /* 插入事件 */
                usb_hotplug_arrived(dev);
                break;
            case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
                /*拔出事件*/
                usb_hotplug_left(dev);
                break;
            default:
                break;
        }

        dev->event = 0;
        ret = libusb_handle_events(dev->cxt);
        if(ret!=0)
           Debug("usb hotplug event fild %s \n",libusb_error_name(ret));
        
        if(dev->exit_flag == 1)
            break;
    }
    Debug("usb device is left\n");
    return NULL;
    
}

int fill_iobuffer(void *opaque,uint8_t *buf, int bufsize)
{
    int ret;
    usb_device *dev = (usb_device *)opaque;
    Debug("wait.....\n");
    pthread_cond_wait(&dev->read_cond,&dev->read_mutex);
    Debug("get a frame\n");
    if(dev->io_buffer!=NULL && !dev->exit_flag){
        memcpy(buf,dev->io_buffer,dev->len);
        return dev->len;
    }else{
        return -1;
    }
}



void *encode_thread(void *arg)
{
    usb_device *dev = (usb_device *)arg;
    int frist_frame = 1;
    int ret;

    AVFormatContext *p = avformat_alloc_context();
    if(NULL == p){
        Debug("%d\n",AVERROR(ENOMEM));
        return NULL;
    }

    uint8_t *iobuffer = (uint8_t *)malloc(IOBufferMaxSize);
    if(NULL == iobuffer)
        return NULL;

    AVIOContext *avio = avio_alloc_context(iobuffer,IOBufferMaxSize,0,dev,fill_iobuffer,NULL,NULL);
    p->pb = avio;

    ret = avformat_open_input(&p,NULL,NULL,NULL);
    if(ret!= 0){
        Debug("%s\n",av_err2str(ret));
        goto ERR;
    }

    AVCodecContext *de_codec = avcodec_alloc_context3(NULL);
    AVCodecContext *en_codec = avcodec_alloc_context3(NULL);

    ret = avcodec_parameters_to_context(de_codec,p->streams[0]->codecpar);
    if(ret < 0){
        Debug("%s\n",av_err2str(ret));
        goto ERR;
    }

    AVCodec *de_code = (AVCodec *)avcodec_find_decoder(de_codec->codec_id);
    if((ret = avcodec_open2(de_codec,de_code,NULL))!= 0){
        Debug("%s\n",av_err2str(ret));
        goto ERR;
    }

    AVFormatContext *f = NULL;
    ret = avformat_alloc_output_context2(&f,NULL,NULL,"./frame.mp4");
    if(ret < 0){
        Debug("%s\n",av_err2str(ret));
        goto ERR;
    }

    AVStream *st = avformat_new_stream(f,NULL);

    st->time_base = p->streams[0]->time_base;
    AVPacket *out_pack = av_packet_alloc();
    AVPacket *in_pack = av_packet_alloc();
    AVFrame *out_frame = av_frame_alloc();
    int read_end = 0;
    while (1)
    {
        if(!dev->exit_flag || read_end)
            break;
        
        ret = av_read_frame(p,in_pack);
        if(ret == 0){
            if(in_pack->stream_index == 1){
                av_packet_unref(in_pack);
                continue;
            }

            ret = avcodec_send_packet(en_codec,in_pack);

            if(ret!= 0){
                Debug("%s\n",av_err2str(ret));
            }
            av_packet_unref(in_pack);
        }else if(ret < 0){
            avcodec_send_packet(de_codec,NULL);
        }else{
            Debug("%s\n",av_err2str(ret));
            continue;
        }

        for(;;){

            ret = avcodec_receive_frame(de_codec,out_frame);
            if(AVERROR_EOF == ret){
                ret = avcodec_send_packet(en_codec,NULL);
                read_end = 1;
                break;
            }else if (ret == 0){
                if(NULL == en_codec){
                    AVCodec *en_avcode = (AVCodec*)avcodec_find_encoder(AV_CODEC_ID_H264);
                    en_codec = avcodec_alloc_context3(en_avcode);

                    en_codec->codec_type = AVMEDIA_TYPE_VIDEO;
                    en_codec->bit_rate = 4000000;
                    en_codec->framerate = de_codec->framerate;
                    en_codec->gop_size = 50;
  //                  en_codec->max_b_frames = 5;
                    en_codec->profile = FF_PROFILE_H264_HIGH_444;

                    en_codec->time_base = p->streams[0]->time_base;
                    en_codec->width = p->streams[0]->codecpar->width;
                    en_codec->height = p->streams[0]->codecpar->height;
                    en_codec->sample_aspect_ratio = out_frame->sample_aspect_ratio;
                    
                    en_codec->pix_fmt = out_frame->format;

                    en_codec->color_primaries = out_frame->color_primaries;
                    en_codec->color_range = out_frame->color_range;
                    en_codec->color_trc = out_frame->color_trc;
                    en_codec->colorspace = out_frame->colorspace;
                    en_codec->chroma_sample_location = out_frame->chroma_location;
                    
                    en_codec->field_order = AV_FIELD_UNKNOWN;

                    ret = avcodec_parameters_from_context(st->codecpar,en_codec);
                    if(0 > ret)
                    {
                        Debug("%s\n",av_err2str(ret));
                        goto ERR;
                    }
                    /*打开编码器*/
                    if(0 > (ret = avcodec_open2(en_codec,en_avcode,NULL)))
                    {
                       Debug("%s\n",av_err2str(ret));
                        goto ERR;
                    }
                  //  av_dict_free(&options);
                    /*正式打开输出文件*/
                    if(0 >(ret = avio_open2(&f->pb,"./fream.mp4",AVIO_FLAG_WRITE,&f->interrupt_callback,NULL)))
                    {
                        Debug("%s\n",av_err2str(ret));
                        goto ERR;
                    }
                    /*写入数据头到输出文件*/
                    if(0 > (ret = avformat_write_header(f,NULL)))
                    {
                        Debug("%s\n",av_err2str(ret));
                        goto ERR;
                    }

                    out_pack->stream_index = st->index;
                    out_pack->pts = av_rescale_q_rnd(out_pack->pts,p->streams[0]->time_base,st->time_base,AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX);
                    out_pack->dts = av_rescale_q_rnd(out_pack->dts,p->streams[0]->time_base,st->time_base,AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX);
                    out_pack->duration = av_rescale_q_rnd(out_pack->duration,p->streams[0]->time_base,st->time_base,AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX);
                    /*把数据包写入输出文件*/
                    ret = av_interleaved_write_frame(f,out_pack);
                    if(0!=ret)
                    {
                        Debug("%s\n",av_err2str(ret));
                        goto ERR;
                    }
                    av_packet_unref(out_pack);
                }

            }else if(AVERROR(EAGAIN) == ret){
                break;
            }else{
                Debug("%s\n",av_err2str(ret));
            }
            
        }
    }
    

    ERR:
        avio_context_free(&avio);
        avformat_free_context(p);
        free(iobuffer);

        avcodec_free_context(&de_codec);
        avcodec_free_context(&en_codec);
        return NULL;
}

/**
 * @brief 
 * usb库初始化,注册热插拔回调,创建热插拔事件处理线程
 * @param dev 
 * @return int 
 * success return 1
 * fail return
 */
int usb_init(usb_device *dev)
{
    int ret;
    int vendor_id = LIBUSB_HOTPLUG_MATCH_ANY;
    int product_id = LIBUSB_HOTPLUG_MATCH_ANY;
    int device_class_id = LIBUSB_HOTPLUG_MATCH_ANY;
    if(NULL == dev)
        return -1;

    memset(dev,0,sizeof(usb_device));

    ret = pthread_mutex_init(&dev->mutex,NULL);
    if(ret)
    {
         Debug("%s %d pthread_mutex init error!\n",__func__,__LINE__);
         return -1;
    }

    if((ret = libusb_init(&dev->cxt))!=0)
    {
        Debug("%s %d err is %s\n",__func__,__LINE__,libusb_error_name(ret));
        return -1;
    }

    if(!(ret = libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)))
    {
        Debug("当前库不支持热插拔!\n");
        goto InitErr;
    }
    /*注册回调 响应热插拔事件*/
    ret = libusb_hotplug_register_callback(dev->cxt,
        LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, /*响应的事件类型*/
        LIBUSB_HOTPLUG_ENUMERATE,              /*如果传入0 只有发生热插拔事件时 才会响应  传入LIBUSB_HOTPLUG_ENUMERATE标志初始化之前已经插入的设备 也会响应*/
        vendor_id,                             /*指定监控的 vendor_id 如果为LIBUSB_HOTPLUG_MATCH_ANY  则不判断vendor_id*/
        product_id,                            /*指定监控的 product_id 如果为LIBUSB_HOTPLUG_MATCH_ANY  则不判断 product_id*/
        device_class_id,                       /*指定监控的 dev_class  如果为LIBUSB_HOTPLUG_MATCH_ANY  则不判断 dev_class 这里的class与libusb_device_descriptor匹配*/
        usb_hotplug_callback,dev,&dev->hotplug_handle);

    if(ret!=LIBUSB_SUCCESS)
    {
        Debug("%s %d err is %s\n",__func__,__LINE__,libusb_error_name(ret));
        goto InitErr;
    }
    /*创建事件处理线程*/
    ret = pthread_create(&dev->event_thread_id,NULL,usb_event_monitor_process,dev);

    pthread_cond_init(&dev->read_cond,NULL);
    pthread_mutex_init(&dev->read_mutex,NULL);
    ret = pthread_create(&dev->event_thread_id,NULL,encode_thread,dev);

    return 1;

    InitErr:
        libusb_exit(dev->cxt);
        pthread_mutex_destroy(&dev->mutex);
        return -1;
}



int main(void)
{
    int ret;
    usb_device *dev = (usb_device *)malloc(sizeof(usb_device));
    if(NULL == dev)
        return -1;

    if((ret = usb_init(dev)) < 0){
        Debug("usb device err\n");
        free(dev);
        return -1;
    }

    while (!exit_flag){
        ;
    }

    free(dev->usb_transfer);
    free(dev);
    return 0;
}
