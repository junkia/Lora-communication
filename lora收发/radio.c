#include <math.h>
#include <string.h>
#include <stdio.h>
#include "src/radio.h"


uint32_t spi_mode = SPI_MODE_0;
uint8_t spi_bits_per_word = 8;
uint32_t spi_speed = 1000000;
uint8_t tx_buf[128], rx_buf[128];

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64
#define RESET_PIN     129// m4对应24       m3对应129         // RESET引脚对应的GPIO引脚编号

/*--SPI相关操作-*/
void dump(const uint8_t *data, uint16_t len)
{
	int i;

    for (i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

void transfer(uint8_t const *tx, uint8_t const *rx, size_t len)
{
  int ret;
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = len,
      .delay_usecs = 0,
      .speed_hz = spi_speed,
      .bits_per_word = spi_bits_per_word,
      .tx_nbits = 1,
      .rx_nbits = 1
  };

  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0) {
    printf("can't send spi message\n");
  }
}

/*-----gpio------*/
// 导出 GPIO 引脚
void gpio_export(unsigned int gpio) {
    int fd, len;
    char buf[MAX_BUF];

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
        //perror("gpio/export");
        printf("GPIO%d read error: gpio_export\n", gpio);
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);

}

// GPIO设置为输出模式
void set_gpio_output(int pin)
{
    char buf[MAX_BUF];
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", pin);

    int fd = open(buf, O_WRONLY);
    if (fd < 0) {
        //perror("gpio/direction");
        printf("GPIO%d read error: set_gpio_output\n", pin);
        return;
    }

    write(fd, "out", 4);
    close(fd);
}

// GPIO设置为输入模式
void set_gpio_input(int pin)
{
    char buf[MAX_BUF];
    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", pin);
    
    int fd = open(buf, O_WRONLY);
    if (fd < 0) {
        //perror("gpio/direction");
        printf("GPIO%d read error: set_gpio_input\n", pin);
        return;
    }

    write(fd, "in", 3);
    
    close(fd);
}

// GPIO设置输出值
void write_gpio_value(unsigned int gpio, unsigned int value)
{
    int fd , len;
    char buf[MAX_BUF];

    len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("gpio/set-value");
    }

    if (value)
        write(fd, "1", 2);
    else
        write(fd, "0", 2);

    close(fd);
}

// 延迟指定的毫秒数
void delay_ms(unsigned int milliseconds)
{
    usleep(milliseconds * 1000);
}

 /*------------------llcc68寄存器读写移植--------------------*/

llcc68_hal_status_t llcc68_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    //printf("write command: ");

    memcpy(tx_buf, command, command_length);
    memcpy(tx_buf + command_length, data, data_length);

    transfer(tx_buf, rx_buf, command_length + data_length);
    //dump(tx_buf, command_length + data_length);

    return LLCC68_HAL_STATUS_OK;
}

llcc68_hal_status_t llcc68_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    //printf("read command: ");
    //dump(command, command_length);

    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    memcpy(tx_buf, command, command_length);
    transfer(command, rx_buf, command_length + data_length);

    memcpy(data, rx_buf + command_length, data_length);
    //printf("read data: ");
    //dump(data, data_length);

    return LLCC68_HAL_STATUS_OK;
}

llcc68_hal_status_t llcc68_hal_reset( const void* context )
{
    
    gpio_export(RESET_PIN);
    // 将RESET引脚设置为输出模式
    set_gpio_output(RESET_PIN);
    // 拉低RESET引脚，复位LLCC68设备
    write_gpio_value(RESET_PIN, 0);
    delay_ms(10);
    // 将RESET引脚设置为输入模式
    set_gpio_input(RESET_PIN);
    delay_ms(20);

    printf("reset\n");
    return LLCC68_HAL_STATUS_OK;
}

llcc68_hal_status_t llcc68_hal_wakeup( const void* context )
{
    printf("wakeup\n");
    return LLCC68_HAL_STATUS_OK;
}

 /*------------------程序初始化--------------------*/

void spi_init()
{
        spi_fd = open("/dev/spidev2.0", O_RDWR);//m4为/dev/spidev1.0   m3为/dev/spidev2.0
    if (spi_fd < 0) {
        printf("failed opening SPI device %d", spi_fd);
    }
    ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
}

void llcc68_init()
{
    llcc68_reset(&llcc68);
    llcc68_hal_wakeup(&llcc68);
    llcc68_set_standby(&llcc68, LLCC68_STANDBY_CFG_RC);
    llcc68_set_pkt_type(&llcc68, LLCC68_PKT_TYPE_LORA);

    llcc68_set_rf_freq(&llcc68, 434000000L);   
    
    pa_params.hp_max = 5;          // output power
    pa_params.pa_duty_cycle = 3;   // is +20 dBm
    pa_params.device_sel = 0;
    pa_params.pa_lut = 1;
    llcc68_set_pa_cfg(&llcc68, &pa_params);
    llcc68_set_tx_params(&llcc68, 20, LLCC68_RAMP_3400_US);//根据手册得出
    //llcc68_set_tx_params(&llcc68,20,200);
    llcc68_set_buffer_base_address(&llcc68, 0, 128);

    //llcc68_cal_img(&llcc68,0x68,0x6F);  //r430-440MHz

    //llcc68_cal_img(&llcc68,0x75,0x81);  //470-510MHz

    mod_params.bw = LLCC68_LORA_BW_250;
    mod_params.sf = LLCC68_LORA_SF10;
    mod_params.cr = LLCC68_LORA_CR_4_5;
    mod_params.ldro = 0;
    llcc68_set_lora_mod_params(&llcc68, &mod_params);
    printf("set_lora_mod_params  success\n");
    //llcc68_get_irq_status(&llcc68, &irq_status);
    pkt_params.crc_is_on = true;
    pkt_params.invert_iq_is_on = false;
    pkt_params.preamble_len_in_symb = 8;//前导码长度8
    pkt_params.header_type = LLCC68_LORA_PKT_EXPLICIT; //LLCC68_LORA_PKT_IMPLICIT;
    pkt_params.pld_len_in_bytes = 76;//设置发包长度76字节
    llcc68_set_lora_pkt_params(&llcc68, &pkt_params);
    printf("set_lora_pkt_params  success\n");
    
    cad_parms.cad_detect_min= 10;
    cad_parms.cad_detect_peak =23;//根据sf参数查表设置cad感知系数
    cad_parms.cad_symb_nb = LLCC68_CAD_04_SYMB;
    cad_parms.cad_timeout = 1000000;
    cad_parms.cad_exit_mode = LLCC68_CAD_ONLY;//cad模式
    llcc68_set_cad_params(&llcc68,&cad_parms );

    air_num=llcc68_get_lora_time_on_air_numerator( &pkt_params,&mod_params);
    air_ms=llcc68_get_lora_time_on_air_in_ms( &pkt_params,&mod_params);
    printf("On air = %ld ms [%ld]\n",air_ms,air_num);//获取lora传输空中时间
    //llcc68_get_irq_status(&llcc68, &irq_status);

    int irq_mask = LLCC68_IRQ_ALL;//打开全部中断标志
    //llcc68_get_irq_status(&llcc68, &irq_status);

    llcc68_set_dio_irq_params(&llcc68, irq_mask, irq_mask, 0, 0);

    //llcc68_get_irq_status(&llcc68, &irq_status);

    //llcc68_get_status(&llcc68, &radio_status);
    //llcc68_get_irq_status(&llcc68, &irq_status);

    //adr0x741 0x3444 for Public Network
    //0x1424 for Private Network
    
    llcc68_set_lora_sync_word( &llcc68, 0x12); //0x12为私有，0x34为公有

    llcc68_set_dio2_as_rf_sw_ctrl(&llcc68, true );//dio2引脚控制打开
    llcc68_clear_irq_status(&llcc68, LLCC68_IRQ_ALL);
}