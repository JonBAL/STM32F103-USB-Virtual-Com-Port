/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "hw_config.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_init.h"

#include "USB-FIFO.h"

//#include "stm32f10x_gpio.h"

/* Private typedef -----------------------------------------------------------*/
//EXTI_InitTypeDef EXTI_InitStructure;
GPIO_InitTypeDef port;

/* Private define ------------------------------------------------------------*/
//#define TX_FIFO_SIZE 256
//#define RX_FIFO_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t packet_buffer[64], packet_size;
uint8_t flag_RX_USB = 0;
uint8_t flag_TX_USB = 0;

//FIFO(TX_FIFO_SIZE) tx_fifo;	// Буфер для отправляемых данных
//FIFO(RX_FIFO_SIZE) rx_fifo;	// Буфер для принимаемых данных

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SystemInit
* Description    : Заменяет настройки тактирование из system_stm32f10x.c (там закоментированно)
								 : Перенаправление из startup_stm32f10x_md.s перед запуском _main
* Input          : None.
* Return         : None.
*******************************************************************************/
void SystemInit() // Глобальные настройки тактирования
{
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
 
	/* Конфигурацяи  SYSCLK, HCLK, PCLK2 и PCLK1 */    
	/* Включаем HSE */    
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	 
	/* Ждем пока HSE не выставит бит готовности либо не выйдет таймаут*/
	do 
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;  
	} 
	while( (HSEStatus == 0) && (StartUpCounter != HSEStartUp_TimeOut));
 
	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (uint32_t)0x01;
	}
	else
	{
		HSEStatus = (uint32_t)0x00;
	}
 
	/* Если HSE запустился нормально */
	if ( HSEStatus == (uint32_t)0x01) 
	{
		/* Включаем буфер предвыборки FLASH */
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		/* Конфигурируем Flash на 2 цикла ожидания */
		/* Это нужно потому, что Flash не может работать на высокой частоте */
		FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
		FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

		/* HCLK = SYSCLK (AHB prescaler) */
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1; // Без делителя

		/* PCLK1 = HCLK (APB1, 36 MHz max) */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;	// Делитель на 2
		
		/* PCLK2 = HCLK (APB2, 72 MHz max) */
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1; // Без делителя

		/* Конфигурируем множитель PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
		/* При условии, что кварц на 8МГц! */
		/* RCC_CFGR_PLLMULL9 - множитель на 9. Если нужна другая частота, не 72МГц */
		/* то выбираем другой множитель. */
		
		// 24 * 3 = 72, без предделителей с умножением на 3
		//RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
		
		// Тут была бага...
		//RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL3);

		/* Включаем PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Ожидаем, пока PLL выставит бит готовности */
		while((RCC->CR & RCC_CR_PLLRDY) == 0);

		/* Выбираем PLL как источник системной частоты */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

		/* Ожидаем, пока PLL выберется как источник системной частоты */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08);
	}
	else 
	{
		/* Все плохо... HSE не завелся... Чего-то с кварцем или еще что...
				Надо бы както обработать эту ошибку... Если мы здесь, то мы работаем
				от HSI! */
	}
}

/*******************************************************************************
* Function Name  : Delay_ms
* Description    : Задержка, указывается в мс. Очень низкая точность.
* Input          : None.
* Return         : None.
*******************************************************************************/
//void Delay_ms(uint32_t ms)
//{
//	// В stm32f10x_conf.h установлена часота кварца для рассчёта
//	volatile uint32_t nCount;
//	RCC_ClocksTypeDef RCC_Clocks;
//	RCC_GetClocksFreq (&RCC_Clocks);

//	nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
//	for (; nCount!=0; nCount--);
//}

/******************************************************************************/
int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIOA->CRH &= !(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1);
	GPIOA->CRH = GPIO_CRH_MODE8_1;
	GPIOA->BSRR = GPIO_BSRR_BS8;

	
	Set_System();
	Set_USBClock();

	USB_Interrupts_Config();
	USB_Init();

	while (1)
	{
		packet_buffer_to_rx_fifo();
		rx_fifo_to_tx_fifo();
		tx_fifo_to_packet_buffer();	

		
//			if(GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_8))
//			{
//				GPIO_ResetBits(GPIOA, GPIO_Pin_8);
//			}
//			else
//			{
//				GPIO_SetBits(GPIOA, GPIO_Pin_8);
//			}
	
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed ( uint8_t * file , uint32_t line)
{
// Infinite loop
// Use GDB to find out why we're here
	while (1);
}
#endif
