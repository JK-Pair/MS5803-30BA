/*
This is a library for the MS5803-30BA presurre sensor. The sensor can operate on both I2C and SPI communication, but for this library only support I2C communication.
For more details, see in the datasheet below.
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5803-30BA&DocType=Data+Sheet&DocLang=English
*/

#ifndef __MS580330BA_H__
#define __MS580330BA_H__

#include "Arduino.h"
#include "math.h"
// #include "WireBase.h"
#include "Wire.h"

// MS5803-30BA Address A 
#define MS5803_ADDRESS_A 0x76
// MS5803-30BA Address B
#define MS5803_ADDRESS_B 0x77
#define RESET_CMD 0x1E
#define ADC_READ 0x00

/*
Initial paramertes
Pressure resolution (D1) OSR - 1024 = 1 mbar
Temperature resolution (D2) OSR - 1024 = 0.005 degs
*/

class MS580330BA{
    public:
        MS580330BA(uint8_t address_ = MS5803_ADDRESS_A, uint16_t pressure_res = 1024, uint16_t temp_res = 1024);
        void initialSensor();
        void resetSensor();
        bool init_status;
        void sensorCalculation();
        int32_t getPressure();
        int32_t getTemperature();
        int32_t requestData(uint8_t cmd_);

    private:
        uint32_t D1 = 0;
        uint32_t D2 = 0;
        int32_t dT = 0;
        int32_t TEMP = 0;
        int64_t OFF = 0;
        int64_t SENS = 0;
        int32_t T2 = 0;
        int32_t OFF2 = 0;
        int32_t SENS2 = 0;
        byte MSB = 0x00;
        byte LSB = 0x00;
        byte LSB2 = 0x00;
        uint8_t sensorAddr;
        int32_t tempData = 0;
        int32_t pressureData = 0;
        uint8_t pressureResolution;
        uint8_t tempResolution;
        uint16_t calibrationCoeff[8];
        unsigned char ms5803CRC(uint16_t n_prom[]);

};

#endif /*__MS580330BA_H__*/