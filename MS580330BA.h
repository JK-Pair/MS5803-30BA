/*
This is a library for the MS5803-30BA presurre sensor. The sensor can operate on both I2C and SPI communication, but for this library only support I2C communication.
For more details, see in the datasheet below.
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5803-30BA&DocType=Data+Sheet&DocLang=English
*/

#ifndef __MS580330BA_H__
#define __MS580330BA_H__

#include "Arduino.h"
// #include "math.h"
// #include "WireBase.h"
// #include "Wire.h"
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
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
enum measurement
{
	TEMPERATURE,
	PRESSURE,
};

enum MATH_POW
{
    POW_2_4 = 16,
    POW_2_7 = 128,
	POW_2_8 = 256,
	POW_2_13 = 8192,
	POW_2_15 = 32768,
	POW_2_16 = 65536,
	POW_2_21 = 2097152,
	POW_2_23 = 8388608,
	POW_2_33 = 8589934592,
	POW_2_37 = 137438953472,
};

class MS580330BA{
    public:
        MS580330BA(uint8_t address_ = MS5803_ADDRESS_A, uint16_t pressure_res = 1024, uint16_t temp_res = 1024, TwoWire *theWire = &Wire);
        void initialSensor();
        void resetSensor();
        bool init_status;
        void getD1Value();
        void getD2Value();
        void sensorCalculation();
        void requestData(measurement sensor_= PRESSURE, bool Timer_=false);
        float getPressure();
        float getTemperature();
        uint32_t getRawData();

    private:
        bool useTimer;
        uint32_t D1 = 0;
        uint32_t D2 = 0;
        uint8_t sensorAddr;
        float tempData = 0;
        float pressureData = 0;
        uint8_t pressureCommand;
        uint8_t tempCommand;
        uint16_t calibrationCoeff[8];
        unsigned char ms5803CRC(uint16_t n_prom[]);

        bool writeI2C(uint8_t address_);
        bool readI2C(uint8_t address_, byte *buffer, byte size_);
        Adafruit_I2CDevice *i2c_pressure = NULL; ///< Pointer to I2C bus interface

};

#endif /*__MS580330BA_H__*/