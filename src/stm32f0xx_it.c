/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f0xx_it.h"
#include "pins.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
#define I2C_BYTE_TO_SEND (0xAA)
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}
//
///**
//  * @brief  This function handles I2C1 interrupt request.
//  * @param  None
//  * @retval None
//  */
//void I2C1_IRQHandler(void)
//{
//	/* USER CODE BEGIN I2C1_IRQn 0 */
//	TEST_PORT->BSRR |= TEST_PIN;		//debug test
//
//	/* USER CODE END I2C1_IRQn 0 */
//	if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
//		HAL_I2C_ER_IRQHandler(&hi2c1);
//	} else {
//		HAL_I2C_EV_IRQHandler(&hi2c1);
//	}
//	/* USER CODE BEGIN I2C1_IRQn 1 */
//	/* USER CODE END I2C1_IRQn 1 */
//
//
//
//	uint32_t I2C_InterruptStatus = I2C1->ISR; // Get interrupt status //
//
////	if((I2C_InterruptStatus & I2C_ISR_ADDR) == I2C_ISR_ADDR) // Check address match //
////	{
////		I2C1->ICR |= I2C_ICR_ADDRCF; // Clear address match flag //
////		if((I2C1->ISR & I2C_ISR_DIR) == I2C_ISR_DIR) // Check if transfer direction is read (slave transmitter) //
////		{
////			I2C1->CR1 |= I2C_CR1_TXIE; // Set transmit IT //
////		}
////	}
////	else if((I2C_InterruptStatus & I2C_ISR_TXIS) == I2C_ISR_TXIS)
////	{
////		I2C1->CR1 &=~ I2C_CR1_TXIE; // Disable transmit IT //
////		I2C1->TXDR = I2C_BYTE_TO_SEND; // Byte to send //
////	}
////	else
////	{
////		GPIOC->BSRR = GPIO_BSRR_BS_8; // Lit orange LED //
////		NVIC_DisableIRQ(I2C1_IRQn); // Disable I2C1_IRQn //
////	}
//
//	TEST_PORT->BRR |= TEST_PIN;		//debug test
//}
