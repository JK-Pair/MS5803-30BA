/*
This is a library for the MS5803-30BA presurre sensor. The sensor can operate on both I2C and SPI communication, but for this library only support I2C communication.
For more details, see in the datasheet below.
https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5803-30BA&DocType=Data+Sheet&DocLang=English
*/
// #include "Wire.h"

#include "MS580330BA.h"
/*!
 *  @brief  Instantiates a new MS580330BA class
 *  @param  address_
 *          i2c address
 *          MS5803_ADDRESS_A = 0x76
 *          MS5803_ADDRESS_B = 0x77
 *  @param  pressure_res
 *          Pressure resolution @256, 512, 1024, 2048, 4096
 *  @param  temp_res
 *          Temperature resolution @256, 512, 1024, 2048, 4096
 */

MS580330BA::MS580330BA(uint8_t address_, uint16_t pressure_res, uint16_t temp_res, TwoWire *theWire){
    sensorAddr = address_;
    i2c_pressure = new Adafruit_I2CDevice(sensorAddr, theWire);

    init_status = false;
    bool pressureBool = false;
    bool tempBool = false;

    for (int res=1; res<=16; res*=2){
        if (pressure_res == (res * 256)){
            pressureBool = true;

        }if (temp_res == (res * 256)){
            tempBool = true;
        }
    }
    init_status = pressureBool && tempBool;
    pressureCommand = 0x40 + (pressure_res / 256); //@256 hex value is 0x40, the next is +2 (0x42)
    tempCommand = 0x50 + (temp_res / 256);
}

void MS580330BA::initialSensor(){
    /*To read the content of the calibration PROM and to calculate the calibration coefficients.
        PROM Read: 0xA0 to 0xAE
    */
    if(!i2c_pressure->begin()){
        init_status = false;
    }
    i2c_pressure->setSpeed(400000);
    resetSensor();

    for (byte addr=0; addr<8; addr++){
        uint8_t buffer[2];
        readI2C((0xA0 + (addr * 2)), buffer, 2);
        calibrationCoeff[addr] = (buffer[0] << 8) + buffer[1];
    }

    unsigned char memoryCRC = calibrationCoeff[7];
    unsigned char calculateCRC = ms5803CRC(calibrationCoeff);
    if ((memoryCRC == calculateCRC && (calibrationCoeff[1] != 0))){
        init_status &= true;
    }else{
        init_status &= false;
    }
}

bool MS580330BA::i2cDetect(){
    return i2c_pressure->detected();
}
bool MS580330BA::writeI2C(uint8_t address_){
    uint8_t buffer[1] = {address_};
    return i2c_pressure->write(buffer,2);
}

bool MS580330BA::readI2C(byte address_, uint8_t *buffer, uint8_t size_){
    uint8_t writeBuffer[1] = {address_};
    return i2c_pressure->write_then_read(writeBuffer, 1, buffer, size_);
}

void MS580330BA::resetSensor(){
    writeI2C(RESET_CMD);
    delay(10);
}

unsigned char MS580330BA::ms5803CRC(uint16_t n_prom[]){
    /*! This function from https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5803-30BA&DocType=Data+Sheet&DocLang=English
    *   @param  n_prom
        calibration coefficient that read from PROM memory of the sensor
    */
   
    int cnt; // simple counter
    unsigned int n_rem; // crc reminder
    unsigned int crc_read; // original value of the crc
    unsigned char n_bit;
    n_rem = 0x00;
    crc_read=n_prom[7]; //save read CRC
    n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0

    for (cnt = 0; cnt < 16; cnt++){ // operation is performed on bytes
        // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

        for (n_bit = 8; n_bit > 0; n_bit--){
            if (n_rem & (0x8000)){
                n_rem = (n_rem << 1) ^ 0x3000;
            }else{
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
    n_prom[7]=crc_read; // restore the crc_read to its original place
    return (n_rem ^ 0x00);
}

void MS580330BA::requestData(measurement sensor_, bool Timer_){
    /*!
        @param sensor_
        command for selecting the pressure or temperature data from the sensor
        @param Timer_
        Timer status, (Default False), If it true mean this function called by Timer
    */
    //Send the command to the chip
    // resetSensor();
    uint8_t cmd_;

    if(sensor_ == TEMPERATURE){
        cmd_ = tempCommand;
    }else{
        cmd_ = pressureCommand;
    }
    useTimer = Timer_;
    // Wire.beginTransmission(sensorAddr);
    // Wire.write(cmd_);
    // Wire.endTransmission();
    writeI2C(cmd_);

    //Define the delay as a response time in the figure.1 in datasheet
    //0.5 / 1.1 / 2.1 / 4.1 /8.22
    if(!useTimer){
        switch (cmd_ & 0x0f){
            case 0:
                delay(1);
                break;
            case 2:
                delay(2);
                break;
            case 4:
                delay(3);
                break;
            case 6:
                delay(5);
                break;
            case 8:
                delay(9);
                break;
        }
    }
}

uint32_t MS580330BA::getRawData(){
    uint32_t result = 0;
    uint8_t buffer[3];
    readI2C(ADC_READ, buffer, 3); 
    result = ((uint32_t)buffer[0] << 16) + ((uint16_t)buffer[1] << 8) + buffer[2];;
    return result;
}

void MS580330BA::getD1Value(){
    D1 = getRawData(); //D1 is a digital pressure value
}
void MS580330BA::getD2Value(){
    D2 = getRawData(); //D2 is a digital temperature value
}

void MS580330BA::sensorCalculation(){

    if(!useTimer){
        requestData(PRESSURE, pressureCommand); 
        D1 = getRawData(); //D1 is a digital pressure value
        requestData(TEMPERATURE, tempCommand);
        D2 = getRawData(); //D2 is a digital temperature value
    }

    int32_t dT =  D2 - (calibrationCoeff[5] * POW_2_8);
    int32_t TEMP = 2000 + ((int64_t)dT * calibrationCoeff[6]) / POW_2_23;

    int64_t OFF = (calibrationCoeff[2] * POW_2_16) + ((calibrationCoeff[4] * (int64_t)dT) / POW_2_7);
    int64_t SENS = (calibrationCoeff[1] * POW_2_15) + ((calibrationCoeff[3] * (int64_t)dT) / POW_2_8);

    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000){
        T2 = 3 * ((int64_t)dT * (int64_t)dT) / POW_2_33;
        OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 8;

        if (TEMP < -1500){
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500));
        }

    }else{
        T2 = 7 * ((int64_t)dT * (int64_t)dT) / POW_2_37;
        OFF2 = 1 * ((TEMP - 2000) * (TEMP - 2000)) / POW_2_4;
        SENS2 = 0;
    }

    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    pressureData = (D1 * SENS / POW_2_21 - OFF) / POW_2_13;
    tempData = TEMP;

    ///Reset parameters
    D1 = 0;
    D2 = 0;
    dT = 0;
    TEMP = 0;
    OFF = 0;
    SENS = 0;
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
    /////////////////////////////////////
}
float MS580330BA::getPressure(){
    int32_t P_MIN = 0; //mbar
    int32_t P_MAX = 30000; //bar
    
    pressureData /= 10; //mbar
    if ((pressureData >= P_MIN) && (pressureData <= P_MAX)){
        return pressureData; //mbar
    }else{
        return 99.0;
    }
}
float MS580330BA::getTemperature(){
    int8_t T_MIN = -40; //celsius 
    uint8_t T_MAX = 85; //celsius 
    tempData /= 100; //Celsius
    if ((tempData >= T_MIN) && (tempData <= T_MAX)){
        return tempData; //Celsius
    }else{
        return 99.0;
    }
}