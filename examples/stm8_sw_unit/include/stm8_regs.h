// stm8_regs.h
// Minimal register-level definitions for STM8S003F3
// Keeps only common/useful peripherals
// cSpell: disable

#pragma once
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// =======================
// GPIO (General Purpose)
// =======================
typedef struct {
  volatile uint8_t ODR; // Output Data
  volatile uint8_t IDR; // Input Data
  volatile uint8_t DDR; // Data Direction
  volatile uint8_t CR1; // Control Register 1
  volatile uint8_t CR2; // Control Register 2
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef*)0x5000)
#define GPIOB ((GPIO_TypeDef*)0x5005)
#define GPIOC ((GPIO_TypeDef*)0x500A)
#define GPIOD ((GPIO_TypeDef*)0x500F)

// =======================
// GPIO Arduino-style aliases
// =======================

// Port A
#define PA_ODR   (GPIOA->ODR)
#define PA_IDR   (GPIOA->IDR)
#define PA_DDR   (GPIOA->DDR)
#define PA_CR1   (GPIOA->CR1)
#define PA_CR2   (GPIOA->CR2)

// Port B
#define PB_ODR   (GPIOB->ODR)
#define PB_IDR   (GPIOB->IDR)
#define PB_DDR   (GPIOB->DDR)
#define PB_CR1   (GPIOB->CR1)
#define PB_CR2   (GPIOB->CR2)

// Port C
#define PC_ODR   (GPIOC->ODR)
#define PC_IDR   (GPIOC->IDR)
#define PC_DDR   (GPIOC->DDR)
#define PC_CR1   (GPIOC->CR1)
#define PC_CR2   (GPIOC->CR2)

// Port D
#define PD_ODR   (GPIOD->ODR)
#define PD_IDR   (GPIOD->IDR)
#define PD_DDR   (GPIOD->DDR)
#define PD_CR1   (GPIOD->CR1)
#define PD_CR2   (GPIOD->CR2)


// =======================
// Clock (CLK)
// =======================
#define CLK_ICKR (*(volatile uint8_t*)0x50C0)     // Internal Clock Control Register
#define CLK_ECKR (*(volatile uint8_t*)0x50C1)     // External Clock Control Register
#define CLK_CMSR (*(volatile uint8_t*)0x50C3)     // Clock Master Status Register
#define CLK_SWR (*(volatile uint8_t*)0x50C4)      // Switch Register
#define CLK_SWCR (*(volatile uint8_t*)0x50C5)     // Switch Control Register
/**
 * @brief Clock Divider Register. This register contains two bit fields.
 *  The HSIDIV field is used to divide the High-Speed Internal (HSI) clock, while the 
 *  CPUDIV field is used to divide the selected clock source (HSI or HSE) for the CPU clock. 
 *  The combined divisor is 2^n, where n = 0..10.
 * @note HSIDIV (bits 3:4) 0   0   0   1   1   0   0   0    =  0x18 (CLK_CKDIVR_HSIDIV)
 * @note CPUDIV (bits 0:2) 0   0   0   0   0   1   1   1    =  0x07 (CLK_CKDIVR_CPUDIV)
 * @attention HSIDIV applies only to HSI. If you switch to HSE, HSIDIV is ignored. The effects UART, I2C etc.
 * @warning CPUDIV only affects the CPU clock, not the peripheral clocks. Peripheral clocks are controlled by PCKENR1 and PCKENR2.
 */
#define CLK_CKDIVR (*(volatile uint8_t*)0x50C6)   
#define CLK_PCKENR1 (*(volatile uint8_t*)0x50C7)  // Peripheral Clock Gating Register 1
#define CLK_CSSR (*(volatile uint8_t*)0x50C8)     // Clock Security System Register
#define CLK_CCOR (*(volatile uint8_t*)0x50C9)     // Clock Calibration Output Register
#define CLK_PCKENR2 (*(volatile uint8_t*)0x50CA)  // Peripheral Clock Gating Register 2
#define CLK_HSITRIMR (*(volatile uint8_t*)0x50CC) // High-Speed Internal Trimming Register
#define CLK_SWIMCCR (*(volatile uint8_t*)0x50CD)  // SWIM Clock Control Register

#define CLK_ICKR_SWUAH ((uint8_t)0x20)  /*!< Slow Wake-up from Active Halt/Halt modes */
#define CLK_ICKR_LSIRDY ((uint8_t)0x10) /*!< Low speed internal oscillator ready */
#define CLK_ICKR_LSIEN ((uint8_t)0x08)  /*!< Low speed internal RC oscillator enable */
#define CLK_ICKR_FHWU ((uint8_t)0x04)   /*!< Fast Wake-up from Active Halt/Halt mode */
#define CLK_ICKR_HSIRDY ((uint8_t)0b10) /*!< High speed internal RC oscillator ready */
#define CLK_ICKR_HSIEN ((uint8_t)0x01)  /*!< High speed internal RC oscillator enable */

#define CLK_ECKR_HSERDY ((uint8_t)0b10) /*!< High speed external crystal oscillator ready */
#define CLK_ECKR_HSEEN ((uint8_t)0x01)  /*!< High speed external crystal oscillator enable */

#define CLK_CMSR_CKM ((uint8_t)0xFF) /*!< Clock master status bits */

#define CLK_SWR_SWI ((uint8_t)0xFF) /*!< Clock master selection bits */

#define CLK_SWCR_SWIF ((uint8_t)0x08)  /*!< Clock switch interrupt flag */
#define CLK_SWCR_SWIEN ((uint8_t)0x04) /*!< Clock switch interrupt enable */
#define CLK_SWCR_SWEN ((uint8_t)0x02)  /*!< Switch start/stop */
#define CLK_SWCR_SWBSY ((uint8_t)0x01) /*!< Switch busy flag*/

#define CLK_CKDIVR_HSIDIV ((uint8_t)0b00011000) /*!< High speed internal clock prescaler bit mask*/
#define CLK_CKDIVR_CPUDIV ((uint8_t)0b00000111) /*!< CPU clock prescaler bit mask */

// -----------------------------
// CLK Peripheral Clock Enable
// -----------------------------

// PCKENR1 bits
#define CLK_PCKENR1_TIM2 (1 << 5)  // TIM2 clock enable  (PCKEN15, 0x20)
#define CLK_PCKENR1_TIM4 (1 << 4)  // TIM4 clock enable  (PCKEN14, 0x10)
#define CLK_PCKENR1_UART1 (1 << 3) // UART1 clock enable (PCKEN13, 0x08)
#define CLK_PCKENR1_SPI (1 << 1)   // SPI clock enable   (PCKEN11, 0x02)
#define CLK_PCKENR1_I2C (1 << 0)   // I2C clock enable   (PCKEN10, 0x01)

// PCKENR2 bits
#define CLK_PCKENR2_ADC (1 << 3) // ADC clock enable   (PCKEN23, 0x08)
#define CLK_PCKENR2_AWU (1 << 2) // AWU clock enable   (PCKEN22, 0x04)

// CMSR values
#define CLK_SRC_HSI 0xE1 ///  @brief Clock source register, High-Speed Internal oscillator
#define CLK_SRC_LSI 0xD2 ///  @brief Clock source register, Low-Speed Internal oscillator
#define CLK_SRC_HSE 0xB4 ///  @brief Clock source register, High-Speed External oscillator
#define CLK_SRC_LSE 0xD1 ///  @brief Clock source register, Low-Speed External oscillator

#define CLK_CSSR_CSSD        ((uint8_t)0x08) /*!< Clock security system detection */
#define CLK_CSSR_CSSDIE      ((uint8_t)0x04) /*!< Clock security system detection interrupt enable */
#define CLK_CSSR_AUX         ((uint8_t)0x02) /*!< Auxiliary oscillator connected to master clock */
#define CLK_CSSR_CSSEN       ((uint8_t)0x01) /*!< Clock security system enable */


// =======================
// UART1
// =======================
#define UART1_SR (*(volatile uint8_t*)0x5230)
#define UART1_DR (*(volatile uint8_t*)0x5231)
#define UART1_BRR1 (*(volatile uint8_t*)0x5232)
#define UART1_BRR2 (*(volatile uint8_t*)0x5233)
#define UART1_CR1 (*(volatile uint8_t*)0x5234)
#define UART1_CR2 (*(volatile uint8_t*)0x5235)
#define UART1_CR3 (*(volatile uint8_t*)0x5236)
#define UART1_CR4 (*(volatile uint8_t*)0x5237)
#define UART1_CR5 (*(volatile uint8_t*)0x5238)
#define UART1_GTR (*(volatile uint8_t*)0x5239)
#define UART1_PSCR (*(volatile uint8_t*)0x523A)

// UART1_SR flags
#define UART1_SR_TXE (1 << 7)  // Transmit data register empty
#define UART1_SR_TC (1 << 6)   // Transmission complete
#define UART1_SR_RXNE (1 << 5) // Received data ready
#define UART1_SR_IDLE (1 << 4)
#define UART1_SR_OR (1 << 3)
#define UART1_SR_NF (1 << 2)
#define UART1_SR_FE (1 << 1)
#define UART1_SR_PE (1 << 0)

// UART1_SR flags
#define UART_SR_TXE UART1_SR_TXE   // Transmit data register empty
#define UART_SR_TC UART1_SR_TC     // Transmission complete
#define UART_SR_RXNE UART1_SR_RXNE // Received data ready
#define UART_SR_IDLE UART1_SR_IDLE
#define UART_SR_OR UART1_SR_OR
#define UART_SR_NF UART1_SR_NF
#define UART_SR_FE UART1_SR_FE
#define UART_SR_PE UART1_SR_PE

// UART1_CR2 bits
#define UART_CR2_TIEN (1 << 7) // TX interrupt enable
#define UART_CR2_TCIEN (1 << 6)
#define UART_CR2_RIEN (1 << 5)  // RX interrupt enable
#define UART_CR2_ILIEN (1 << 4) // IDLE line interrupt enable
#define UART_CR2_TEN (1 << 3)   // Transmitter enable
#define UART_CR2_REN (1 << 2)   // Receiver enable
#define UART_CR2_RWU (1 << 1)   // Receiver wakeup
#define UART_CR2_SBK (1 << 0)

/*
 * UART1 Control Register 2 (UART1_CR2) Bit Definitions:
 *
 * UART1_CR2_TIE  - Transmit Interrupt Enable (bit 7)
 *                  Enables interrupt generation when the transmit data register is empty.
 *
 * UART1_CR2_TCIE - Transmission Complete Interrupt Enable (bit 6)
 *                  Enables interrupt generation when transmission of a frame is complete.
 *
 * UART1_CR2_RIE  - Receiver Interrupt Enable (bit 5)
 *                  Enables interrupt generation when the receive data register is not empty.
 *
 * UART1_CR2_ILIE - IDLE Line Interrupt Enable (bit 4)
 *                  Enables interrupt generation when an idle line is detected.
 *
 * UART1_CR2_TEN  - Transmitter Enable (bit 3)
 *                  Enables the UART transmitter.
 *
 * UART1_CR2_REN  - Receiver Enable (bit 2)
 *                  Enables the UART receiver.
 *
 * UART1_CR2_RWU  - Receiver Wakeup (bit 1)
 *                  Selects mute mode or normal mode for the receiver.
 *
 * UART1_CR2_SBK  - Send Break (bit 0)
 *                  Forces the transmission of a break character.
 */
// UART1 Control Register 2
#define UART1_CR2_TIE (1 << 7)
#define UART1_CR2_TCIE (1 << 6)
#define UART1_CR2_RIE (1 << 5)
#define UART1_CR2_ILIE (1 << 4)
#define UART1_CR2_TEN (1 << 3)
#define UART1_CR2_REN (1 << 2)
#define UART1_CR2_RWU (1 << 1)
#define UART1_CR2_SBK (1 << 0)

// -------------------------
// UART1_CR3 bits
// -------------------------
#define UART1_CR3_STOP1 (1 << 4)
#define UART1_CR3_STOP2 (1 << 5)

// =======================
// Timer2 (16-bit general)
// =======================
#define TIM2_CR1 (*(volatile uint8_t*)0x5300)
#define TIM2_IER (*(volatile uint8_t*)0x5303)
#define TIM2_SR1 (*(volatile uint8_t*)0x5304)
#define TIM2_EGR (*(volatile uint8_t*)0x5306)
#define TIM2_CCMR1 (*(volatile uint8_t*)0x5307)
#define TIM2_CCMR2 (*(volatile uint8_t*)0x5308)
#define TIM2_CCMR3 (*(volatile uint8_t*)0x5309)
#define TIM2_CCER1 (*(volatile uint8_t*)0x530A)
#define TIM2_CNTRH (*(volatile uint8_t*)0x530C)
#define TIM2_CNTRL (*(volatile uint8_t*)0x530D)
#define TIM2_PSCR (*(volatile uint8_t*)0x530E)
#define TIM2_ARRH (*(volatile uint8_t*)0x530F)
#define TIM2_ARRL (*(volatile uint8_t*)0x5310)
#define TIM2_CCR1H (*(volatile uint8_t*)0x5311)
#define TIM2_CCR1L (*(volatile uint8_t*)0x5312)

// TIM2_CR1 bits
#define TIM2_CR1_CEN (1 << 0)  // (0x01) Counter enable
#define TIM2_CR1_UDIS (1 << 1) // Update disable
#define TIM2_CR1_URS (1 << 2)  // Update request source
#define TIM2_CR1_OPM (1 << 3)  // One pulse mode
#define TIM2_CR1_DIR (1 << 4)  // 0x10 Direction
#define TIM2_CR1_CMS0 (1 << 5) // 0x20 Center-aligned mode bit 0
#define TIM2_CR1_CMS1 (1 << 6) // Center-aligned mode bit 1
#define TIM2_CR1_ARPE (1 << 7) // Auto-reload preload enable

// TIM2_CR1
#define TIM_CR1_CEN TIM2_CR1_CEN // (0x01) Counter enable
#define TIM_CR1_UDIS TIM2_CR1_UDIS
#define TIM_CR1_URS TIM2_CR1_URS
#define TIM_CR1_OPM TIM2_CR1_OPM
#define TIM_CR1_DIR TIM2_CR1_DIR
#define TIM_CR1_CMS TIM2_CR1_CMS1
#define TIM_CR1_ARPE TIM2_CR1_ARPE

// TIM2_IER / SR1
#define TIM2_IER_UIE (1 << 0) // (0x01) Update interrupt enable
#define TIM2_SR1_UIF (1 << 0) // (0x01) Update interrupt flag

#define TIM_IER_UIE TIM2_IER_UIE
#define TIM_SR1_UIF TIM2_SR1_UIF

// TIM2_EGR
#define TIM2_EGR_UG (1 << 0) // (0x01) Update generation

// =======================
// Timer4 (8-bit basic)
// =======================
#define TIM4_CR1 (*(volatile uint8_t*)0x5340)
#define TIM4_IER (*(volatile uint8_t*)0x5343)
#define TIM4_SR (*(volatile uint8_t*)0x5344)
#define TIM4_EGR (*(volatile uint8_t*)0x5345)
#define TIM4_CNTR (*(volatile uint8_t*)0x5346)
#define TIM4_PSCR (*(volatile uint8_t*)0x5347)
#define TIM4_ARR (*(volatile uint8_t*)0x5348)

// CR1
#define TIM4_CR1_CEN (1 << 0) // (0x01) Counter enable
// IER/SR
#define TIM4_IER_UIE (1 << 0) //
#define TIM4_SR_UIF (1 << 0)

// EGR
#define TIM4_EGR_UG (1 << 0) // (0x01) Update generation

// =======================
// External Interrupts
// =======================
#define EXTI_CR1 (*(volatile uint8_t*)0x50A0)
#define EXTI_CR2 (*(volatile uint8_t*)0x50A1)

// =======================
// Less-used / Special
// =======================

// Reset status
#define RST_SR (*(volatile uint8_t*)0x50B3)
#define RST_SR_WWDGF (1 << 0)
#define RST_SR_IWDGF (1 << 1)
#define RST_SR_ILLOPF (1 << 2)
#define RST_SR_SWIMF (1 << 3)
#define RST_SR_EMCF (1 << 4)

// Flash
#define FLASH_CR1 (*(volatile uint8_t*)0x505A)
#define FLASH_CR2 (*(volatile uint8_t*)0x505B)
#define FLASH_NCR2 (*(volatile uint8_t*)0x505C)
#define FLASH_FPR (*(volatile uint8_t*)0x505D)
#define FLASH_NFPR (*(volatile uint8_t*)0x505E)
#define FLASH_IAPSR (*(volatile uint8_t*)0x505F)
#define FLASH_PUKR (*(volatile uint8_t*)0x5062)
#define FLASH_DUKR (*(volatile uint8_t*)0x5064)

#define FLASH_CR1_RESET_VALUE   ((uint8_t)0x00)
#define FLASH_CR2_RESET_VALUE   ((uint8_t)0x00)
#define FLASH_NCR2_RESET_VALUE  ((uint8_t)0xFF)
#define FLASH_IAPSR_RESET_VALUE ((uint8_t)0x40)
#define FLASH_PUKR_RESET_VALUE  ((uint8_t)0x00)
#define FLASH_DUKR_RESET_VALUE  ((uint8_t)0x00)

// Option bytes
#define OPT0 (*(volatile uint8_t*)0x4800)
#define OPT1 (*(volatile uint8_t*)0x4801)
#define OPT2 (*(volatile uint8_t*)0x4802)
#define OPT3 (*(volatile uint8_t*)0x4803)
#define OPT4 (*(volatile uint8_t*)0x4804)
#define OPT5 (*(volatile uint8_t*)0x4805)
#define OPT6 (*(volatile uint8_t*)0x4806)
#define OPT7 (*(volatile uint8_t*)0x4807)
