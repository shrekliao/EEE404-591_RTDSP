/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "gpio.h"
#include "timer.h"

int main(void)
{
	// LED GPIO configuration
	// Discovery kit with STM32F407VG MCU User manual: hardware and layout, LEDs
	// User LD4: green LED is a user LED connected to the I/O PD12 of the STM32F407VGT6

	// assign GPIO port base address
	GPIO_RegDef_t *pGPIOLed;
	pGPIOLed = GPIOD;

	// assign pin number
	uint8_t GPIO_PinNumber_Led = GPIO_PIN_NO_12;

    // reset
	GPIOD_REG_RESET();

	// enable peripheral clock
	GPIOD_PCLK_EN();

	// configure the LED peripheral register
	//1 . configure the mode of GPIO pin: output
	pGPIOLed->MODER &= ~( 3UL << (2 * GPIO_PinNumber_Led)); //clearing bits, UL represents unsigned long integer with 32 bits
	pGPIOLed->MODER |= (GPIO_MODE_OUT << (2 * GPIO_PinNumber_Led ) ); //setting to desired value

	//2. configure the output type: push-pull
	pGPIOLed->OTYPER &= ~( 1UL << GPIO_PinNumber_Led); //clearing bit
	pGPIOLed->OTYPER |= (GPIO_OP_TYPE_PP << GPIO_PinNumber_Led ); //setting to desired value

	//3. configure the output speed: low speed
	pGPIOLed->OSPEEDR &= ~( 3UL << ( 2 * GPIO_PinNumber_Led)); //clearing bits
	pGPIOLed->OSPEEDR |= (GPIO_SPEED_LOW << ( 2 * GPIO_PinNumber_Led) ); //setting to desired value

	//4. configure the pull-up/pull-down setting: no pull-up/pull-down
	pGPIOLed->PUPDR &= ~( 3UL << ( 2 * GPIO_PinNumber_Led)); //clearing bits
	pGPIOLed->PUPDR |= (GPIO_NO_PUPD << ( 2 * GPIO_PinNumber_Led) ); //setting to desired value


    // assign TIM6 base address
	TIMx_RegDef_t *pTIM6;
	pTIM6 = TIM6;

    // reset
	TIM6_REG_RESET();

	// enable TIM6 clock
	TIM6_PCLK_EN();

    // configure TIM6 registers
	// This is the place where you change timer period.
	// The current period is 0.5 second, i.e., LED 0.5 sec on, 0.5 sec off
	pTIM6->PSC = 29999; // set prescaler
	pTIM6->ARR = 799; // set auto reload register
	//change

	pTIM6->CR1 |= 1 << 0; // set bit 0 CEN to enable timer

    // configure interrupt
	//1. Enable TIM6 interrupt
	pTIM6->DIER |= 1 << 0; // set bit 0 UIE to enable update interrupt

	//2. Enable TIM6_DAC interrupt on NVIC controller
	//program ISER1 register (IRQ_NO_TIM6_DAC)54 - 32 = 22
	*NVIC_ISER1 |= ( 1 << (IRQ_NO_TIM6_DAC-32) );//SFRs IDE is kind of wrong

	/* Loop forever */
	for(;;);
}
// Interrupt Handler
// The function name of the interrupt handler is given by the startup assembly file startup_stm32f407vgtx.s
void TIM6_DAC_IRQHandler(void)
{
	if ( (TIM6->SR & ( 1 << 0)) != 0) // check bit 0 UIF flag to see if interrupt occurred
	{
		// toggle GPIO pin by writing 0/1 to the GPIO port output data register
		GPIOD->ODR  ^= ( 1UL << GPIO_PIN_NO_12);

		//clear the SR register bit 0 UIF flag
		TIM6->SR &= ~( 1 << 0);
	}
}
