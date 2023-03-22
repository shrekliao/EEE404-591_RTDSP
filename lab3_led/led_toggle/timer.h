/*
 * timer.h
 *
 *  Created on: Feb 8, 2021
 *      Author: cwang135
 */

#ifndef TIMER_H_
#define TIMER_H_

#define __vo volatile

/*
 * STM32F4 memory map
 * STM32F4 reference manual: STM32F4xx register boundary addresses
 */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define APB1PERIPH_BASEADDR						0x40000000U

#define TIM6_BASEADDR                   (APB1PERIPH_BASEADDR + 0x1000)

/*
 * This is a peripheral register definition structure for TIM6 & TIM7
 * STM32F4 reference manual: TIM registers
 */
typedef struct
{
	__vo uint16_t CR1;		/*!< TIMx_CR1,	Address offset: 0x00	*/
	uint16_t RESERVED0;		/*!< Reserved,	Address offset: 0x02	*/
	__vo uint16_t CR2;		/*!< TIMx_CR2,	Address offset: 0x04	*/
	uint16_t RESERVED1;		/*!< Reserved,	Address offset: 0x06	*/
	uint32_t RESERVED2;		/*!< Reserved,	Address offset: 0x08	*/
	__vo uint16_t DIER;		/*!< TIMx_DIER,	Address offset: 0x0C	*/
	uint16_t RESERVED3;		/*!< Reserved,	Address offset: 0x0E	*/
	__vo uint16_t SR;		/*!< TIMx_SR,	Address offset: 0x10	*/
	uint16_t RESERVED4;		/*!< Reserved,	Address offset: 0x12	*/
	__vo uint16_t EGR;		/*!< TIMx_EGR,	Address offset: 0x14	*/
	uint16_t RESERVED5;		/*!< Reserved,	Address offset: 0x16	*/
	uint32_t RESERVED6[3];		/*!< Reserved,	Address offset: 0x18-0x20	*/
	__vo uint16_t CNT;		/*!< TIMx_CNT,	Address offset: 0x24	*/
	uint16_t RESERVED7;		/*!< Reserved,	Address offset: 0x26	*/
	__vo uint16_t PSC;		/*!< TIMx_PSC,	Address offset: 0x28	*/
	uint16_t RESERVED8;		/*!< Reserved,	Address offset: 0x2A	*/
	__vo uint16_t ARR;		/*!< TIMx_ARR,	Address offset: 0x2C	*/
	uint16_t RESERVED9;		/*!< Reserved,	Address offset: 0x2E	*/
} TIMx_RegDef_t;

#define TIM6  				((TIMx_RegDef_t*)TIM6_BASEADDR)

#define TIM6_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)

#define TIM6_PCLK_EN()    	(RCC->APB1ENR |= (1 << 4))

#define IRQ_NO_TIM6_DAC 		54

#endif /* TIMER_H_ */
