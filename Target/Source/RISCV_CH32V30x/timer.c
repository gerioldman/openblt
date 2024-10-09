/************************************************************************************//**
* \file         Source/_template/timer.c
* \brief        Bootloader timer driver source file.
* \ingroup      Target__template_timer
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2019  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/************************************************************************************//**
* \defgroup   Target__template_timer Timer driver of a port
* \brief      This module implements the timer memory driver of a microcontroller port. 
* \details    The timer driver implements a polling based 1 millisecond timer. It 
*             provides the time base for all timing related parts of the bootloader. The
*             bootloader calls the function TimerUpdate() continuously to check if the
*             next millisecond period passed.
* \ingroup    Target__template
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#include "ch32v30x.h"                            /* Peripheral libraries               */


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Local variable for storing the number of milliseconds that have elapsed since
 *         startup.
 */
static blt_int32u millisecond_counter;
static blt_int16u free_running_counter_last;


/************************************************************************************//**
** \brief     Initializes the polling based millisecond timer driver.
** \return    none.
**
****************************************************************************************/
void TimerInit(void)
{
  /* Reset the timer configuration. */
  TimerReset();

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM7, ENABLE );

  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  uint32_t TIM_Prescaler = (RCC_Clocks.PCLK1_Frequency / 1000u) -1u;
  uint16_t TIM_ClockDivision = TIM_CKD_DIV1;
  if (TIM_Prescaler > 0xFFFFu)
  {
    TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_Prescaler = ((TIM_Prescaler + 1u) / 2u) -1u;
    if (TIM_Prescaler > 0xFFFFu)
    {
      TIM_ClockDivision = TIM_CKD_DIV4;
      TIM_Prescaler = ((TIM_Prescaler + 1u) / 2u) -1u;
    }
  }

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
    /* TIM_Prescaler */         TIM_Prescaler,
    /* TIM_CounterMode */       TIM_CounterMode_Up,
    /* TIM_Period */            0xFFFF,
    /* TIM_ClockDivision */     TIM_ClockDivision,
    /* TIM_RepetitionCounter */ 0u /* Value is don't care for TIM2 */
  };
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
  TIM_Cmd (TIM7, ENABLE);

  /* Reset the millisecond counter value. */
  millisecond_counter = 0;
  free_running_counter_last = TIM_GetCounter(TIM7);
} /*** end of TimerInit ***/


/************************************************************************************//**
** \brief     Reset the timer by placing the timer back into it's default reset
**            configuration.
** \return    none.
**
****************************************************************************************/
void TimerReset(void)
{
  TIM_DeInit(TIM7);
} /* end of TimerReset */


/************************************************************************************//**
** \brief     Updates the millisecond timer.
** \return    none.
**
****************************************************************************************/
void TimerUpdate(void)
{
  /* Increment the millisecond counter. */
  uint16_t counter_difference = TIM_GetCounter(TIM7) - free_running_counter_last;
  free_running_counter_last = TIM_GetCounter(TIM7);
  millisecond_counter += counter_difference;
} /*** end of TimerUpdate ***/


/************************************************************************************//**
** \brief     Obtains the counter value of the millisecond timer.
** \return    Current value of the millisecond timer.
**
****************************************************************************************/
blt_int32u TimerGet(void)
{
  /* Updating timer here allows this function to be called in a loop with timeout
   * detection.
   */
  TimerUpdate();
  /* Read and return the amount of milliseconds that passed since initialization. */
  return millisecond_counter;
} /*** end of TimerGet ***/


/*********************************** end of timer.c ************************************/
