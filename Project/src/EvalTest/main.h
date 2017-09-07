#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32F0xx.h"
#include "core_cm0.h"

/* Exported constants --------------------------------------------------------*/
extern const uint8_t TX_PACKET_1[24];


/* Exported macros -----------------------------------------------------------*/
#define  PLL_MUL_X  3
#define TX_PACKET_1_SIZE (sizeof(TX_PACKET_1))

/* typedef -------------------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* function prototypes -------------------------------------------------------*/
TestStatus BufferCmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void delay(uint32_t ms_time);
void delay_ms(uint32_t n_ms);
void delay_10us(uint32_t n_10us);
void Enable_Usb_1k5_Pullup(void);
void TimingDelay_Decrement(void);

#endif
