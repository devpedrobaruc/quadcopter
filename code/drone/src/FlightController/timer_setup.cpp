#include "FlightController/FlightController.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// More information can be found in these two videos:
// STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
// STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FlightController::timer_setup(void)
{
  // Timer2.attachCompare1Interrupt(handler_channel_1);
  // TIM2->CR1 = TIM_CR1_CEN;
  // TIM2->CR2 = 0;
  // TIM2->SMCR = 0;
  // TIM2->DIER = TIM_DIER_CC1IE;
  // TIM2->EGR = 0;
  // TIM2->CCMR1 = TIM_CCMR1_CC1S_INPUT_TI1;
  // TIM2->CCMR2 = 0;
  // TIM2->CCER = TIM_CCER_CC1E;

  // //TIM2->CCER |= TIM_CCER_CC1P;    //Detect falling edge.
  // TIM2->CCER &= ~TIM_CCER_CC1P; //Detect rising edge.
  // TIM2->PSC = 71;
  // TIM2->ARR = 0xFFFF;
  // TIM2->DCR = 0;

  TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
  TIM4->CR2 = 0;
  TIM4->SMCR = 0;
  TIM4->DIER = 0;
  TIM4->EGR = 0;
  TIM4->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE | (0b110 << 12) | TIM_CCMR1_OC2PE;
  TIM4->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE | (0b110 << 12) | TIM_CCMR2_OC4PE;
  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM4->PSC = 71;
  TIM4->ARR = 5000;
  TIM4->DCR = 0;
  TIM4->CCR1 = 1000;

  TIM4->CCR1 = 1000;
  TIM4->CCR2 = 1000;
  TIM4->CCR3 = 1000;
  TIM4->CCR4 = 1000;
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);
}
