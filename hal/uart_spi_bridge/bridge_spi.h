#include "wiced_memory.h"
#include "wiced_spi.h"
#include "slist.h"

#ifndef REG32
#define REG32(x)     *((volatile unsigned*)(x))
#endif

typedef enum
{
    SPI_INIT_STATE,
    SPI_TX_HEADER_STATE,
    SPI_TX_WAIT_FOR_HEADER_TRANSMIT_DONE_STATE,
    SPI_TX_PAYLOAD_WAIT_FOR_DEV_READY_STATE,
    SPI_TX_WAIT_FOR_PAYLOAD_TRANSMIT_DONE_STATE,
    SPI_RX_WAIT_FOR_CMD_TRANSMIT_DONE_STATE,
    SPI_RX_WAIT_FOR_DATA_READ_START_STATE,
    SPI_TX_DONE_STATE,
    SPI_RX_DONE_STATE,
}spi_master_states_t;

typedef struct
{
     slist_node_t node; /* slist node to handle the spi tx data q*/
     uint8_t* p_data;
     uint32_t length;
}spi_master_data_t;

typedef void (*spi_master_data_rx_cb_t)( uint8_t* p_data );

typedef struct
{
    uint8_t spi_state;
    uint8_t rx_pending;
    uint8_t audio_pending;
    uint16_t rx_packet_length;
    uint8_t* p_rx_buf;
    uint16_t tx_packet_length;
    uint8_t* p_tx_data;
    slist_node_t* p_tx_node;
    wiced_bt_buffer_pool_t* p_spi_rx_buff_pool;
    wiced_bt_buffer_pool_t* p_spi_tx_buff_pool;
    spi_master_data_rx_cb_t p_data_rx_cb;
    struct slist_node_t spi_tx_data_q;
    struct slist_node_t spi_rx_data_q;
}spi_master_info_t;

/* External Definitions */
extern void spi_master_init( void );
extern void spi_master_register_data_received_cb( spi_master_data_rx_cb_t p_cb );
extern wiced_result_t spi_master_tx_data( uint8_t* p_data, uint32_t length );
extern void spi_master_tx_audio_data( uint8_t packet_count );
