#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <linux/spi/spidev.h>

#include "llcc68_hal.h"
#include "llcc68.h"

int spi_fd ;

typedef struct {
} llcc68_context_t;


llcc68_rx_buffer_status_t rx_buffer_status;
llcc68_context_t llcc68;
llcc68_irq_mask_t irq_status;

void dump(const uint8_t *data, uint16_t len);
void transfer(uint8_t const *tx, uint8_t const *rx, size_t len);

llcc68_hal_status_t llcc68_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );
llcc68_hal_status_t llcc68_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );
llcc68_hal_status_t llcc68_hal_reset( const void* context );
llcc68_hal_status_t llcc68_hal_wakeup( const void* context );                             

 /*------------------程序初始化--------------------*/
    llcc68_pa_cfg_params_t pa_params;
    llcc68_mod_params_lora_t mod_params;
    llcc68_pkt_params_lora_t pkt_params;
    llcc68_cad_params_t cad_parms;
    uint32_t air_num,air_ms;



void spi_init();
void llcc68_init();