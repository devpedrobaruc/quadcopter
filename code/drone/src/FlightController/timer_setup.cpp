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
  // TIM2->CR1 = TIMER_CR1_CEN;
  // TIM2->CR2 = 0;
  // TIM2->SMCR = 0;
  // TIM2->DIER = TIMER_DIER_CC1IE;
  // TIM2->EGR = 0;
  // TIM2->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  // TIM2->CCMR2 = 0;
  // TIM2->CCER = TIMER_CCER_CC1E;

  // // TIM2->CCER |= TIMER_CCER_CC1P;    //Detect falling edge.
  // TIM2->CCER &= ~TIMER_CCER_CC1P; // Detect rising edge.
  // TIM2->PSC = 71;
  // TIM2->ARR = 0xFFFF;
  // TIM2->DCR = 0;

  Timer3 = new HardwareTimer(TIM3);
  Timer3->setMode(1, TIMER_INPUT_CAPTURE_FALLING, PA6);
  Timer3->attachInterrupt([this]()
                          { Serial_input_handler(); });

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
