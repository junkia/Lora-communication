#include "src/llcc68.h"
#include "src/radio.h"

#include <stdio.h>
#include <fcntl.h>
//#include <asm-generic/fcntl.h>
#include <unistd.h>
//#include <sys/ioctl.h>
#include <signal.h>
#include <pthread.h>

typedef struct  {
        int Sensors_Type;// 传感器类型   0->无， 1->CO2  ， 2->H2O2  ， 3->温湿度
        int Value;//参数
    }SensorData;

typedef struct {
        int header;
        SensorData sendData[8];
        int tail;
        int verify;
} DataPacket;

void randData(DataPacket dataPacket)
{
    // 创建一个 DataPacket 结构体

    // 使用随机数生成器为 sendData 成员随机赋值
    srand(time(NULL)); // 使用当前时间作为随机数生成器的种子

    int i , j=240;
    for ( i = 0; i < 8; i++) {
        dataPacket.sendData[i].Sensors_Type = 0;
        dataPacket.sendData[i].Value = 0;
    }
    dataPacket.header = 0x0AA0;
    dataPacket.tail = 0x0BB0;
    dataPacket.verify = 0x0CC0;
}
/*DataPacket send_buf = {
        .sendDatamonitorId = 875001,
        .status = 1,
        .probeStatus = 1,
        .sensorCount = 1,
        .signal_strength = 1,
        .electricity = 1,
        .Sensors_Number = 10,
        .Sensors_Type = 0,
        .Activation = 1,
        .Online = 1,
        .Alarm = 1,
        .Up_Limit = 1000,
        .Ultra_Up_Limit = 1500,
        .Low_Limit = 500,
        .Ultra_Low_Limit = 300,
        .Position =11,
        .Value = 24313943
    };*/

//uint8_t send_buf[6] = {'875002','o','n',5,3,4};
//const char send_buf[6]  = "12345";
char  back_buf[] = "back";
uint8_t received_buf[256];
uint16_t rssi;
llcc68_status_t status;
int count=0;

void send(DataPacket *dataPacket)
{
    // 将结构体数据复制到一个 uint8_t 数组中
    uint8_t buffer[sizeof(DataPacket)];
    memcpy(buffer, dataPacket, sizeof(DataPacket));

    size_t buf_length = sizeof(DataPacket) ;
    llcc68_write_buffer(&llcc68, 0, buffer, buf_length);
    llcc68_set_tx(&llcc68, 3000);
    printf("Send data: ");
    dump(buffer, buf_length);

}

void receive()
{
    llcc68_get_rx_buffer_status(&llcc68,&rx_buffer_status);
    //uint8_t rx_buf[256]; // 足够大的缓冲区来存储接收的数据
    uint16_t rx_len = rx_buffer_status.pld_len_in_bytes;
    llcc68_read_buffer(&llcc68, rx_buffer_status.buffer_start_pointer, received_buf, rx_len);
    printf("Received data: ");
    dump(received_buf, rx_len);
    // 获取 RSSI 值
    status = llcc68_get_rssi_inst(&llcc68, &rssi);
    if (status == LLCC68_STATUS_OK) {
        printf("RSSI in dBm: %hd\n", rssi);
    } else {
        printf("Failed to get RSSI\n");
    }

    int pipe_fd;
    pipe_fd = open("/mnt/mypipe", O_RDWR);//提前创建的命名管道用于lora接收与qt程序消息传递
    if (pipe_fd == -1) {
        perror("cannot open the FIFO");
    }
    if(write(pipe_fd,received_buf, rx_len) == -1)  /*写入消息*/
    {
	    perror("write data error!");
    }
    close(pipe_fd);

}

// 定义回调函数
void myCallback(int signum) {
    //printf("Callback function called.\n");
                //FILE *serialPort = fopen("/dev/tty1", "w"); // 打开串口设备文件

    llcc68_get_irq_status(&llcc68, &irq_status);
    //printf("tx_irq_status: %04X\n",irq_status);

    if(irq_status & LLCC68_IRQ_TX_DONE)
    {
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_TX_DONE);
        printf("Tx done \n");
        //fprintf(serialPort, "Tx done %d\n",count);
        //sleep(1);
    }

    if(irq_status & LLCC68_IRQ_RX_DONE)
    {
        //count++;
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_RX_DONE);
        receive();
        printf("Rx done \n");
        //fprintf(serialPort, "Rx done %d\n",count);
        llcc68_set_rx(&llcc68, 0);
        //printf("Back data ");
        //send(back_buf);
    }

    if(irq_status & LLCC68_IRQ_PREAMBLE_DETECTED)
    {
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_PREAMBLE_DETECTED);
        printf("llcc68: irq preamble detected. \n");
    }

    if(irq_status & LLCC68_IRQ_HEADER_VALID)
    {
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_HEADER_VALID);
        printf("llcc68: irq valid header. \n");
    }
    
    if(irq_status & LLCC68_IRQ_TIMEOUT)
    {
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_TIMEOUT);
        printf("llcc68: irq time out. \n");
    }

    if(irq_status & LLCC68_IRQ_CAD_DETECTED)
    {
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_CAD_DETECTED);
        printf("CAD detected, channel is busy. Waiting...\n");
        sleep(1+rand() % 2);
    }
   if(irq_status & LLCC68_IRQ_CAD_DONE)
    {
        DataPacket send_buf2;
        randData(send_buf2);
        llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_CAD_DONE);
        printf("CAD done, channel is clear. Sending data1...\n");
        send(&send_buf2);
        sleep(2);
        
    }
    
    
    //fclose(serialPort);
}
 
int main(void)
{
    DataPacket send_buf;
    int flags = 0;

    spi_init();
    llcc68_init();

    int fd = open("/dev/loram3_irq_driver", O_RDWR);//m4对应/dev/loram4_irq_driver，m3对应/dev/loram3_irq_driver
    if (fd < 0) {
    perror("Unable to open device");
    return -1;
    }

    // 注册信号处理函数
    signal(SIGIO, myCallback);
    fcntl(fd, F_SETOWN, getpid()); /* 将当前进程的进程号告诉给内核 */
    flags = fcntl(fd, F_GETFD); /* 获取当前的进程状态 */
    fcntl(fd, F_SETFL, flags | FASYNC);/* 设置进程启用异步通知功能 */

    //randData(send_buf);不采用cad时循环发送时打开
    llcc68_set_rx(&llcc68, 0);//设置接收模式时打开
    while(1) {
        //llcc68_set_cad(&llcc68);设置发送时打开
        //send(&send_buf);不采用cad时循环发送时打开
        sleep(1);
    }

    return 0;
}