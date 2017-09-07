/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_exti.h"
/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BSRR_VAL 0x0C00
#define SPINN_SHORT_SYMS (5)
#define SPINN_LONG_SYMS  (19)
#define BUFFER_LENGTH    (20)
#define EOP_IDX          (16)
#define POSITION_1
#define POSITION_2
#define POSITION_3
#define POSITION_4
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;
  uint8_t Data_1,Data_2,Data_3,Data_4,Data_5,Data_6,Data_7;
  uint8_t Previous_data, Current_data, Test_v , Test_v2, Test_v3= 0;
  int DA = 1, idx =0;
  uint8_t rx_buf_idx = 0;
  uint8_t rx_buf[SPINN_SHORT_SYMS*2];
  uint8_t Buffer[20] = 0;
  static uint8_t symbol_table[] = 
{
    0x11, 0x12, 0x14, 0x18, 0x21, 0x22, 0x24, 0x28,
    0x41, 0x42, 0x44, 0x48, 0x03, 0x06, 0x0C, 0x09, 0x60
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
    uint16_t speed = 0;
    uint8_t speed_syms[4];
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
//-----------------------------------------------------------------------------
static void hal_init(void)
{
  /* Enable clock on GPIO pins */
  /* GPIOC Periph clock enable */
}
uint8_t spinn_lookup_sym(uint8_t sym)
{
    for (uint8_t idx = 0; idx < sizeof(symbol_table); idx++)
    {
        if (symbol_table[idx] == sym)
        {
            return idx;
        }
    }
    return sizeof(symbol_table);
}
void spinn_use_data(uint8_t *buf)
{


    /* Decode buffer and get motor data */
    speed_syms[0] = spinn_lookup_sym(buf[2]);
    speed_syms[1] = spinn_lookup_sym(buf[3]);
    speed_syms[2] = spinn_lookup_sym(buf[4]);
    speed_syms[3] = spinn_lookup_sym(buf[5]);
    if (speed_syms[0] >= sizeof(symbol_table) || 
        speed_syms[1] >= sizeof(symbol_table) || 
        speed_syms[2] >= sizeof(symbol_table) || 
        speed_syms[3] >= sizeof(symbol_table))
    {
        /* Received a symbol in error; return early */
        return;
    }
    speed = speed_syms[0] +
            (speed_syms[1] << 4) +
            (speed_syms[2] << 8) +
            (speed_syms[3] << 12);
}

void Receive_data(void);


  int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file*/
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);


     /* Configure PB8, PB9, PB10, PB11, PB12, PB13, PB14 as input mode to receive spike*/
    /* Set PB7 as Input_Ack,and set as output mode */ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7  | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure  PB0, PB1, PB2, PB3, PB4, PB5, PB6, as output mode to send spinnaker-packet*/
    /* Set PB15 as Output_Ack,and set as input mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                         GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; /* Output Mode */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /* Highest speed pins */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   /* Output type push pull */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* Pull up */
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    /* Ensure bits are all reset */
    GPIO_Write(GPIOB, 0);

    /* Ensure level shifters are enabled */
    GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);
    
      
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOA, GPIOB and GPIOE Clocks enable */
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
    
    /* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
    
  /* GPIOB Configuration: Channel 2N and 3N as alternate function push-pull */
  
  /* TIM1 Configuration ---------------------------------------------------
   Generate 7 PWM signals with 4 different duty cycles:
   TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 48 MHz for STM32F0xx devices
   
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
   
   Note: 
    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
    Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
    function to update SystemCoreClock variable value. Otherwise, any configuration
    based on this variable will be incorrect. 
  ----------------------------------------------------------------------- */
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
    TimerPeriod = SystemCoreClock ;
  /* Compute CCR1 value to generate a duty cycle  */
    Channel2Pulse = (uint16_t) (((uint32_t)  (TimerPeriod - 1)) / 20);//5%   0.98ms
    Channel1Pulse = (uint16_t) (((uint32_t) 625 * (TimerPeriod - 1)) / 1000);//6.25%   1.25ms 
    Channel3Pulse = (uint16_t) (((uint32_t)  875 * (TimerPeriod - 1)) / 1000);//8.75%  1.75ms



  /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
    
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 33;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);

    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
  
  
    //-------------------------------------------------------------------
   EXTI_InitTypeDef exti;
   NVIC_InitTypeDef nvic;

   /* Configure external interrupt */

    /* Ensure peripheral clock is configured */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 

    /* Map all pins into interrupt */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource13);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14);

    exti.EXTI_Line = EXTI_Line7 ;// | EXTI_Line8  | EXTI_Line9  | EXTI_Line10 | 
                     //EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Signals are transitions, so rising or falling edge */
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    /* Configure interrupt register */
    nvic.NVIC_IRQChannel = EXTI4_15_IRQn;
    nvic.NVIC_IRQChannelPriority = 5;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
     //----------------------------------------------------------------------    
    Previous_data =  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8 ) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9 ) << 1) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) << 2) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) << 3) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 4) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) << 5) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) << 6);
    //On reset, read the default value in the link

 while (1)
 { 

  while (rx_buf_idx < sizeof(rx_buf))
   {
     /*Read the data in the link*/
     Current_data =  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8 ) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9 ) << 1) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10 ) << 2) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11 ) << 3) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12 ) << 4) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13 ) << 5) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14 ) << 6);
   
     rx_buf[rx_buf_idx] = Current_data ^ Previous_data;    //Decoding
     Previous_data = Current_data;                         //Storing the decoded symbol
    
/* Transmit acknowledge as transition */        
      if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15) > 0)
            {
                GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
            }
            else
            {
                GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
            }
/* Decoding SpiNN-Format Data Packet */
         if (rx_buf[rx_buf_idx] == symbol_table[sizeof(symbol_table)-1])
            {  
                       break;
                
            }
            rx_buf_idx++;
   }

  /* Handle data using method */
            spinn_use_data(&rx_buf[0]);          
       
            rx_buf_idx = 0;

 } 
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//---------------  Interruptions  -------------------//
void EXTI4_15_IRQHandler(void)
{
     if (EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line8); //clear fag
                  Data_1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8 );
    }
        if (EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line9); 
                  Data_2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9 );
    }
        if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line10); 
                  Data_3 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10 );
    }
        if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line11); 
                  Data_4 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11 );
    }
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line12); 
                  Data_5 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12 );
    }
        if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line13); 
        Data_6 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13 );
    }
        if (EXTI_GetITStatus(EXTI_Line14) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line14); 
        Data_7 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14 );
    }
          if (EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line7); 
    
    }
   
}


//---------------  add files  -------------------//
/*void Receive_data(void)
{
    uint8_t rx_buf[SPINN_SHORT_SYMS*2];
    uint8_t rx_buf_idx = 0;
    uint8_t previous_data = 0, current_data = 0;

    /* Reset previous data to ensure state is correct */
/*    previous_data =  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8 ) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9 ) << 1) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) << 2) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) << 3) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 4) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) << 5) +
                    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) << 6);
	
    for (;;)
    {

        while (rx_buf_idx < sizeof(rx_buf))
        {
            /* Get semaphore twice */
            /* Read data into buffer */
/*            current_data =  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8 ) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9 ) << 1) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) << 2) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) << 3) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) << 4) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) << 5) +
                           (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) << 6);
            rx_buf[rx_buf_idx] = current_data ^ previous_data;
            previous_data = current_data;
            /* Transmit acknowledge as transition */
/*            if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15) > 0)
            {
                GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
            }
            else
            {
                GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
            }
            if (rx_buf[rx_buf_idx] == symbol_table[sizeof(symbol_table)-1])
            {
                break;
            }
            rx_buf_idx++;
        }

        /* Handle data using method */
//        spinn_use_data(&rx_buf[0]);

        /* Reset buffer */
/*        rx_buf_idx = 0;
    }
}
 
 
}*/