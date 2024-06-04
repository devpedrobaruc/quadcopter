#include <Arduino.h>
#include <FlightController/FlightController.h>

// HardwareSerial Serial1(PA10, PA9);

FlightController controller;

void irq_handler();

void setup()
{
    // Serial.begin(115200);
    controller.begin();

    HardwareTimer *Timer3 = new HardwareTimer(TIM3);
    Timer3->setMode(1, TIMER_INPUT_CAPTURE_FALLING, PA6);
    Timer3->setPrescaleFactor(71);
    Timer3->setOverflow(0xFFFF);
    Timer3->attachInterrupt(1, irq_handler);
    // TIM3->CR1 = TIM_CR1_CEN;
    // TIM3->CR2 = 0;
    // TIM3->SMCR = 0;
    // TIM3->DIER = TIM_DIER_CC1IE;
    // TIM3->EGR = 0;
    // TIM3->CCMR1 = TIM_CCMR1_CC1S_1; // A6 is serial input.
    // TIM3->CCMR2 = 0;
    // TIM3->CCER = TIM_CCER_CC1E;
    // TIM3->CCER |= TIM_CCER_CC1P; // Detect the faling edge.
    // TIM3->PSC = 71;
    // TIM3->ARR = 0xFFFF;
    // TIM3->DCR = 0;
    Timer3->resume();
    pinMode(PC13, OUTPUT);
}

void loop()
{
    controller.process();
}

void irq_handler()
{
    controller.Serial_input_handler();
}