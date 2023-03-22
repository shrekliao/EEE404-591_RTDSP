/*
 * gpio.h
 *
 *  Created on: Dec 22, 2020
 *      Author: cwang135
 */


#ifndef GPIO_H_
#define GPIO_H_

#define __vo volatile

/**********************************Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 * STM32 Cortex®-M4 MCUs and MPUs programming manual: Interrupt set-enable register x (NVIC_ISERx)
 * NVIC_ISER0 bits 0 to 31 are for interrupt 0 to 31, respectively
 * NVIC_ISER1 bits 0 to 31 are for interrupt 32 to 63, respectively, etc.
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 * STM32 Cortex®-M4 MCUs and MPUs programming manual: Interrupt clear-enable register x (NVIC_ICERx)
 * NVIC_ICER0 bits 0 to 31 are for interrupt 0 to 31, respectively
 * NVIC_ICER1 bits 0 to 31 are for interrupt 32 to 63, respectively
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * STM32F4 memory map
 * STM32F4 reference manual: STM32F4xx register boundary addresses
 */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define AHB1PERIPH_BASEADDR						0x40020000U
#define APB2PERIPH_BASEADDR						0x40010000U

/*
 * Base addresses of peripherals GPIO, RCC which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals EXTI which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)

/*
 * This is a peripheral register definition structure for GPIO
 * STM32F4 reference manual: GPIO registers
 */
typedef struct
{
	__vo uint32_t MODER;		/*!< GPIO port mode register,                    														Address offset: 0x00	*/
	__vo uint32_t OTYPER;		/*!< GPIO port output type register,     																Address offset: 0x04	*/
	__vo uint32_t OSPEEDR;		/*!< GPIO port output speed register, 																	Address offset: 0x08	*/
	__vo uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register, 																Address offset: 0x0C	*/
	__vo uint32_t IDR;			/*!< GPIO port input data register, bit 15:0 contains the input value of the corresponding I/O port,	Address offset: 0x10	*/
	__vo uint32_t ODR;			/*!< GPIO port output data register, bit 15:0 contains the output value of the corresponding I/O port,	Address offset: 0x14	*/
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register,																	Address offset: 0x18	*/
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register,																Address offset: 0x1C	*/
	__vo uint32_t AFR[2];		/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    	Address offset: 0x20-0x24 */
}GPIO_RegDef_t;


/*
 * This is a peripheral register definition structure for RCC
 * STM32F4 reference manual: RCC registers
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register, 1 to reset	Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register, 		Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< TODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * This is a peripheral register definition structure for EXTI
 * STM32F4 reference manual: External interrupt/event controller (EXTI),
 * Vector table for STM32F405xx/07xx and STM32F415xx/17xx, EXTI registers
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Interrupt mask register (EXTI_IMR), 0: Interrupt request from line x is masked, 1 not masked, 										Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< Event mask register (EXTI_EMR), 0: Event request from line x is masked, 1 not masked, 												Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< Rising trigger selection register (EXTI_RTSR), 0: Rising trigger disabled (for Event and Interrupt) for input line, 1 enabled, 		Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< Falling trigger selection register (EXTI_FTSR), 0: Falling trigger disabled (for Event and Interrupt) for input line, 1 enabled, 	Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< Software interrupt event register (EXTI_SWIER),  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< Pending register (EXTI_PR),0: No trigger request occurred, 1 occurred, set by edge event, cleared by programming it to '1', 			Address offset: 0x14 */

}EXTI_RegDef_t;



/*
 * This is a peripheral register definition structure for SYSCFG
 * STM32F4 reference manual: SYSCFG register maps for STM32F405xx/07xx and STM32F415xx/17xx
 * SYSCFG registers for STM32F405xx/07xx and STM32F415xx/17xx
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< memory remap register (SYSCFG_MEMRMP),						Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< peripheral mode configuration register (SYSCFG_PMC),		Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< external interrupt configuration register,					Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!<           							  					Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< Compensation cell control register (SYSCFG_CMPCR), 		Address offset: 0x20      */
} SYSCFG_RegDef_t;





/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 * STM32F4 reference manual: RCC AHB1 peripheral clock enable register (RCC_AHB1ENR), set to 1 to enable, clear to 0 to disable
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for SYSCFG peripheral
 * STM32F4 reference manual: RCC APB2 peripheral clock enable register (RCC_APB2ENR)
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 *  STM32F4 reference manual: RCC AHB1 peripheral reset register, set to 1 to reset, clear to 0 to NOT reset
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) // reset, then return to normal
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0  // 00: input (reset state)
#define GPIO_MODE_OUT 		1  // 01: general purpose output mode
#define GPIO_MODE_ALTFN 	2  // 10: alternative function mode
#define GPIO_MODE_ANALOG 	3  // 11: analog mode
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0  // 0: output push-pull (reset state)
#define GPIO_OP_TYPE_OD   1  // 1: output open-drain


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0 // 00: low speed
#define GPIO_SPEED_MEDIUM		1 // 01: medium speed
#define GPIO_SPEED_FAST			2 // 10: high speed
#define GPOI_SPEED_HIGH			3 // 11: very high speed


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0 // 00: no pull-up, pull-down
#define GPIO_PIN_PU			1 // 01: pull-up
#define GPIO_PIN_PD			2 // 10: pull-down
							  // 11: reserved

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * STM32F4 reference manual: External interrupt/event controller (EXTI)
 * Vector table for STM32F405xx/07xx and STM32F415xx/17xx Position
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#endif /* GPIO_H_ */

