#include <Arduino.h>
#include "MS580330BA.h"
MS580330BA pressureSensor = MS580330BA(MS5803_ADDRESS_A, 1024, 1024);

void setup() {
    Serial.begin(115200);
    pressureSensor.initialSensor();
    if(!pressureSensor.init_status){
        while(1){
            Serial.println("Pressure sensor error");
            delay(100);
        }
    }
}

void loop(){
    pressureSensor.sensorCalculation();
    // Serial.println(pressureSensor.getPressure());
    Serial.println(pressureSensor.getTemperature());

    Serial.println("DONE");
    delay(100);
}