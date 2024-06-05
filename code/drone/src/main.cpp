#include <Arduino.h>
#include <FlightController/FlightController.h>

HardwareSerial Serial1(PA10, PA9);
FlightController controller;

void setup()
{
    Serial.begin(115200);
    controller.begin();
}

void loop()
{
    controller.process();
}