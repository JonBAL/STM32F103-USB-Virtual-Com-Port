/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "fifo.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TX_FIFO_SIZE 256
#define RX_FIFO_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t counter = 0;
uint16_t i = 0;

FIFO(TX_FIFO_SIZE) tx_fifo;	// Буфер для отправляемых данных
FIFO(RX_FIFO_SIZE) rx_fifo;	// Буфер для принимаемых данных

/* Extern variables ----------------------------------------------------------*/
extern int8_t packet_buffer[], packet_size;
extern uint8_t flag_RX_USB;
extern uint8_t flag_TX_USB;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : packet_buffer_to_rx_fifo
* Description    : Перенос в RX_fifo по флагу приёма
* Input          : None.
* Return         : None.
*******************************************************************************/
void packet_buffer_to_rx_fifo (void)
{
	if (flag_RX_USB == 1)
	{
		for(i = 0; i < packet_size; i++)
		{
			if (!FIFO_IS_FULL(rx_fifo))
			{
				FIFO_PUSH (rx_fifo, packet_buffer[i]);
			}
		}
		flag_RX_USB = 0;
	}
}
		
/*******************************************************************************
* Function Name  : rx_fifo_to_tx_fifo
* Description    : Перенос между RF_fifo и TX_fifo
* Input          : None.
* Return         : None.
*******************************************************************************/
void rx_fifo_to_tx_fifo (void)
{
	if (!FIFO_IS_EMPTY(rx_fifo))
	{
		counter = FIFO_COUNT(rx_fifo);
		for (i = 0; i < counter; i++)
		{
			if (FIFO_IS_EMPTY(rx_fifo)) break;

			FIFO_PUSH (tx_fifo, FIFO_FRONT(rx_fifo)+2);
			FIFO_POP(rx_fifo);
		}
	}
}

/*******************************************************************************
* Function Name  : tx_fifo_to_packet_buffer
* Description    : Перенос из TX_fifo в буфер USB
* Input          : None.
* Return         : None.
*******************************************************************************/
void tx_fifo_to_packet_buffer (void)
{
	if (!FIFO_IS_EMPTY(tx_fifo))
	{
		counter = FIFO_COUNT(tx_fifo);
		for (packet_size = 0; packet_size < counter; packet_size++)
		{
			if (FIFO_IS_EMPTY(tx_fifo)) break;
			if (packet_size == 64) break;
			
			packet_buffer[packet_size] = FIFO_FRONT(tx_fifo);
			FIFO_POP(tx_fifo);
		}
		flag_TX_USB = 1;
	}
}
