
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_FIFO_H
#define __USB_FIFO_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void packet_buffer_to_rx_fifo (void);
void rx_fifo_to_tx_fifo (void);
void tx_fifo_to_packet_buffer (void);

/* External variables --------------------------------------------------------*/
#endif  /*__USB_FIFO_H*/
