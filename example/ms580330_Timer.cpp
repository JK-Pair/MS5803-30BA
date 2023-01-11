#include <Arduino.h>
#include "MS580330BA.h"

#define SystemLED PB12 //System LED pin define
#define OperatingLED PB14 //System LED pin define

MS580330BA pressureSensor = MS580330BA(MS5803_ADDRESS_B, 1024, 1024);
uint32 prescaler_ = 720;
uint8 freqPressure = 100; //The target frequency in Hz
uint16 overflowPressure = (72e6 / (freqPressure * prescaler_)) - 1;
uint8_t pressureState = 0;

uint8 freqPrint = 1; //The target frequency in Hz
uint16 overflowPrint = (72e6 / (freqPrint * prescaler_)) - 1;

void handlerPressure(void){

  switch (pressureState)
  {
  case 0:
    pressureSensor.requestData(PRESSURE, true);
    pressureState += 1;
    break;

  case 1:
    pressureSensor.getD1Value();
    pressureSensor.requestData(TEMPERATURE, true);
    pressureState += 1;
    break;

  case 2:
    pressureSensor.getD2Value();
    pressureSensor.sensorCalculation();
    pressureState = 0;
    break;
  }
}

void handlerPrint(void){
  Serial.println(pressureSensor.getTemperature());
}

void setup() {
    Serial.begin(115200);
    pinMode(SystemLED, OUTPUT);
    pinMode(OperatingLED, OUTPUT);

    pressureSensor.initialSensor();
    if(!pressureSensor.init_status){
        while(1){
            Serial.println("Pressure sensor error");
            delay(100);
        }
    }

    // Setup Counting Timers
    Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);

    Timer1.setPrescaleFactor(prescaler_);
    Timer2.setPrescaleFactor(prescaler_);

    Timer1.pause();
    Timer2.pause();

    Timer1.setCount(0);
    Timer2.setCount(0);

    Timer1.setOverflow(overflowPressure);
    Timer2.setOverflow(overflowPrint);

    Timer1.setCompare(TIMER_CH1, overflowPressure/2);   // somewhere in the middle
    Timer2.setCompare(TIMER_CH1, overflowPrint/2);   // somewhere in the middle

    Timer1.attachInterrupt(TIMER_CH1, handlerPressure);
    Timer2.attachInterrupt(TIMER_CH1, handlerPrint);

    Timer1.resume();
    Timer2.resume();
}

void loop(){
  // pressureSensor.sensorCalculation();
  // Serial.print(pressureSensor.getPressure()); Serial.print(", ");
  // Serial.println(pressureSensor.getTemperature());
  // Serial.println("DONE");
  // delay(100);
}