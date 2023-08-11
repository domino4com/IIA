/*!
 * @file IIA.cpp
 * @brief Get data from IIA
 * @n ...
 * @copyright   MIT License
 * @author [Bjarke Gotfredsen](bjarke@gotfredsen.com)
 * @version  V1.0
 * @date  2023
 * @https://github.com/domino4com/IIA
 */
#include "IIA.h"
#define IIA_DEBUG Serial

IIA::IIA() {
}

bool IIA::getData(float &x, float &y, float &z) {
    if (whichIMU() == 1) {  // STMicroelectronics LIS2DH12
        Serial.println("LIS2DH12 - not tested");
        x = getX();
        y = getY();
        z = getZ();
    } else {
        if (whichIMU() == 2) {  // Rohm Kionix KXTJ3-1057
            standby(false);
            uint8_t dataLowRes = 0;
            if (readRegister(&dataLowRes, IIA_XOUT_H) == IMU_SUCCESS) {
                x = axisAccel(Xacc);
            }
            if (readRegister(&dataLowRes, IIA_YOUT_H) == IMU_SUCCESS) {
                y = axisAccel(Yacc);
            }
            if (readRegister(&dataLowRes, IIA_ZOUT_H) == IMU_SUCCESS) {
                z = axisAccel(Zacc);
            }
            standby(true);
        }
    }
    return true;  // Return true for successful read (add error handling if needed)
}

bool IIA::getJSON(JsonObject &doc) {
    float x, y, z;
    if (!getData(x, y, z)) {
        return false;
    }

    JsonArray dataArray = doc.createNestedArray("IIA");

    JsonObject dataSet = dataArray.createNestedObject();  // First data set
    dataSet["name"] = "X";
    dataSet["value"] = x;
    dataSet["unit"] = "g";

    dataSet = dataArray.createNestedObject();   // Subsequent data sets
    dataSet["name"] = "Y";
    dataSet["value"] = y;
    dataSet["unit"] = "g";

    dataSet = dataArray.createNestedObject();   // Subsequent data sets
    dataSet["name"] = "Z";
    dataSet["value"] = z;
    dataSet["unit"] = "g";

    return true;
}

/*
   GNU General Public License
*/

// Begin comm with accel at given I2C address, and given wire port
// Init accel with default settings

int IIA::whichIMU() {
    if (ping(LIS2DH12_ADDR)) return 1;
    if (ping(0x0F)) return 2;
    return 0;
}
bool IIA::ping(int8_t addr) {
    // ping i2c with addr to see if it is there
    int8_t ret = 0;
    Wire.beginTransmission(addr);
    ret = Wire.endTransmission();
    if (ret != 0)
        return false;
    return true;
}

bool IIA::begin(uint8_t range, float sampleRate) {
    switch (whichIMU()) {
        case 0:
            return false;
            break;
        case 1:
            return beginLIS2DH12(range, LIS2DH12_ADDR);
            break;
        case 2:
            if(beginKXTJ3(range, sampleRate, KXTJ3_ADDR)==IMU_SUCCESS) return true;
            return false;
            break;
        default:
            return false;
            break;
    }
}

bool IIA::beginLIS2DH12(uint8_t range, uint8_t i2cAddress) {  // STMicroelectronics LIS2DH12
    Serial.println("LIS2DH12 begin not tested");
    _i2cPort = &Wire;
    _i2cAddress = i2cAddress;  // Capture user's setting
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = (void *)this;
    if (isConnected() == false)
        return false;
    // Enable Block Data Update
    lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    // Set Output Data Rate to 25Hz
    setDataRate(LIS2DH12_ODR_25Hz);
    // Set full scale to 2g
    switch (range) {
        case 0:
            setScale(LIS2DH12_2g);
            break;
        case 1:
            setScale(LIS2DH12_4g);
            break;
        case 2:
            setScale(LIS2DH12_8g);
            break;
        case 3:
            setScale(LIS2DH12_16g);
            break;
        default:
            setScale(LIS2DH12_2g);
            break;
    }
    // Enable temperature sensor
    enableTemperature();
    // Set device in continuous mode with 12 bit resol.
    setMode(LIS2DH12_HR_12bit);
    return true;
}

bool IIA::beginKXTJ3(uint8_t range, float sampleRate, uint8_t i2cAddress) {  // Rohm Kionix IIA-1057

    IIA_status_t returnError = IMU_SUCCESS;
    I2CAddress = i2cAddress;
    accelSampleRate = sampleRate;
    switch (range) {
        case (RANGE_2G):
            accelRange = 2;
            break;
        case (RANGE_4G):
            accelRange = 4;
            break;
        case (RANGE_8G):
            accelRange = 8;
            break;
        case (RANGE_16G):
            accelRange = 16;
            break;
        default:
            accelRange = 2;
            break;
    }
    highRes = false;
    en14Bit = false;
    debugMode = false;
    // Sample rates ≥ 400Hz force High Resolution mode on
    if (accelSampleRate > 200 && !highRes) {
        highRes = true;
    }
    // Perform software reset to make sure IMU is in good state
    returnError = softwareReset();
    // Check previous returnError to see if we should stop
    if (returnError != IMU_SUCCESS) {
        return returnError;
    }
    // Check the ID register to determine if the operation was a success.
    uint8_t _whoAmI;
    readRegister(&_whoAmI, IIA_WHO_AM_I);
    if (_whoAmI != 0x35) {
        return IMU_HW_ERROR;
    }
    // Check the self-test register to determine if the IMU is up.
    uint8_t _selfTest;
    readRegister(&_selfTest, IIA_DCST_RESP);
    if (_selfTest != 0x55) {
        return IMU_HW_ERROR;
    }
    returnError = applySettings();
    return returnError;
}

// Check to see if IC ack its I2C address. Then check for valid LIS2DH ID.
bool IIA::isConnected() {
    _i2cPort->beginTransmission((uint8_t)_i2cAddress);
    if (_i2cPort->endTransmission() == 0) {
        // Something ack'd at this address. Check ID.
        static uint8_t whoamI;
        lis2dh12_device_id_get(&dev_ctx, &whoamI);
        if (whoamI == LIS2DH12_ID) {
            return (true);
        }
    }
    return (false);
}

// Returns true if new data is available
bool IIA::available() {
    lis2dh12_reg_t reg;
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte)
        return true;
    return false;
}

// Blocking wait until new data is available
void IIA::waitForNewData() {
    while (available() == false)
        delay(1);
}

// Returns true if new temperature data is available
bool IIA::temperatureAvailable() {
    lis2dh12_reg_t reg;
    lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte)
        return true;
    return false;
}

// Returns X accel of the global accel data
float IIA::getX() {
    if (xIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    xIsFresh = false;
    return (accelX);
}

// Returns X of the global accel data
int16_t IIA::getRawX() {
    if (xIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    xIsFresh = false;
    return (rawX);
}

// Returns Y accel of the global accel data
float IIA::getY() {
    if (yIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    yIsFresh = false;
    return (accelY);
}

// Returns Y of the global accel data
int16_t IIA::getRawY() {
    if (yIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    yIsFresh = false;
    return (rawY);
}

// Returns Z accel of the global accel data
float IIA::getZ() {
    if (zIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    zIsFresh = false;
    return (accelZ);
}

// Returns Z of the global accel data
int16_t IIA::getRawZ() {
    if (zIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        parseAccelData();
    }
    zIsFresh = false;
    return (rawZ);
}

// Returns sensor temperature in C
float IIA::getTemperature() {
    if (tempIsFresh == false) {
        waitForNewData();  // Blocking wait until available
        getTempData();
    }
    tempIsFresh = false;
    return (temperatureC);
}

// Load global vars with latest accel data
// Does not guarantee data is fresh (ie you can read the same accel values multiple times)
void IIA::parseAccelData() {
    // Read accelerometer data
    axis3bit16_t data_raw_acceleration;
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
    lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

    rawX = data_raw_acceleration.i16bit[0];
    rawY = data_raw_acceleration.i16bit[1];
    rawZ = data_raw_acceleration.i16bit[2];

    // Convert the raw accel data into milli-g's based on current scale and mode
    switch (currentScale) {
        case LIS2DH12_2g:
            switch (currentMode) {
                case LIS2DH12_HR_12bit:  // High resolution
                    accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit:  // Normal mode
                    accelX = lis2dh12_from_fs2_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // Low power mode
                    accelX = lis2dh12_from_fs2_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs2_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs2_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_4g:
            switch (currentMode) {
                case LIS2DH12_HR_12bit:  // High resolution
                    accelX = lis2dh12_from_fs4_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit:  // Normal mode
                    accelX = lis2dh12_from_fs4_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // Low power mode
                    accelX = lis2dh12_from_fs4_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs4_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs4_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_8g:
            switch (currentMode) {
                case LIS2DH12_HR_12bit:  // High resolution
                    accelX = lis2dh12_from_fs8_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit:  // Normal mode
                    accelX = lis2dh12_from_fs8_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // Low power mode
                    accelX = lis2dh12_from_fs8_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs8_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs8_lp_to_mg(rawZ);
                    break;
            }
            break;

        case LIS2DH12_16g:
            switch (currentMode) {
                case LIS2DH12_HR_12bit:  // High resolution
                    accelX = lis2dh12_from_fs16_hr_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_hr_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_hr_to_mg(rawZ);
                    break;
                case LIS2DH12_NM_10bit:  // Normal mode
                    accelX = lis2dh12_from_fs16_nm_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_nm_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_nm_to_mg(rawZ);
                    break;
                case LIS2DH12_LP_8bit:  // Low power mode
                    accelX = lis2dh12_from_fs16_lp_to_mg(rawX);
                    accelY = lis2dh12_from_fs16_lp_to_mg(rawY);
                    accelZ = lis2dh12_from_fs16_lp_to_mg(rawZ);
                    break;
            }
            break;

        default:  // 2g
            accelX = lis2dh12_from_fs2_hr_to_mg(rawX);
            accelY = lis2dh12_from_fs2_hr_to_mg(rawY);
            accelZ = lis2dh12_from_fs2_hr_to_mg(rawZ);
            break;
    }

    xIsFresh = true;
    yIsFresh = true;
    zIsFresh = true;
}

// Load global vars with latest temp data
// Does not guarantee data is fresh (ie you can read the same temp value multiple times)
void IIA::getTempData() {
    // Read temperature data
    axis1bit16_t data_raw_temperature;
    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);

    switch (currentMode) {
        case LIS2DH12_HR_12bit:  // High resolution
            temperatureC = lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
            break;
        case LIS2DH12_NM_10bit:  // Normal mode
            temperatureC = lis2dh12_from_lsb_nm_to_celsius(data_raw_temperature.i16bit);
            break;
        case LIS2DH12_LP_8bit:  // Low power mode
            temperatureC = lis2dh12_from_lsb_lp_to_celsius(data_raw_temperature.i16bit);
            break;
    }

    tempIsFresh = true;
}

// Enter a self test
void IIA::enableSelfTest(bool direction) {
    if (direction == true) {
        lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
    } else {
        lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);
    }
}

// Exit self test
void IIA::disableSelfTest() {
    lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
}

// Set the output data rate of the sensor
void IIA::setDataRate(uint8_t dataRate) {
    if (dataRate > LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP)
        dataRate = LIS2DH12_ODR_25Hz;  // Default to 25Hz
    lis2dh12_data_rate_set(&dev_ctx, (lis2dh12_odr_t)dataRate);
}

// Return the output data rate of the sensor
uint8_t IIA::getDataRate(void) {
    lis2dh12_odr_t dataRate;
    lis2dh12_data_rate_get(&dev_ctx, &dataRate);
    return ((uint8_t)dataRate);
}

// Set full scale of output to +/-2, 4, 8, or 16g
void IIA::setScale(uint8_t scale) {
    if (scale > LIS2DH12_16g)
        scale = LIS2DH12_2g;  // Default to LIS2DH12_2g

    currentScale = scale;  // Used for mg conversion in getX/Y/Z functions

    lis2dh12_full_scale_set(&dev_ctx, (lis2dh12_fs_t)scale);
}

// Return the current scale of the sensor
uint8_t IIA::getScale(void) {
    lis2dh12_fs_t scale;
    lis2dh12_full_scale_get(&dev_ctx, &scale);
    return ((uint8_t)scale);
}

// Enable the onboard temperature sensor
void IIA::enableTemperature() {
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);
}

// Anti-Enable the onboard temperature sensor
void IIA::disableTemperature() {
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);
}

void IIA::setMode(uint8_t mode) {
    if (mode > LIS2DH12_LP_8bit)
        mode = LIS2DH12_HR_12bit;  // Default to 12 bit

    currentMode = mode;

    lis2dh12_operating_mode_set(&dev_ctx, (lis2dh12_op_md_t)mode);
}

// Return the current mode of the sensor
uint8_t IIA::getMode(void) {
    lis2dh12_op_md_t mode;
    lis2dh12_operating_mode_get(&dev_ctx, &mode);
    return ((uint8_t)mode);
}

void IIA::setInt1Threshold(uint8_t threshold) {
    lis2dh12_int1_gen_threshold_set(&dev_ctx, threshold);
}

uint8_t IIA::getInt1Threshold(void) {
    uint8_t threshold;
    lis2dh12_int1_gen_threshold_get(&dev_ctx, &threshold);
    return (threshold);
}

void IIA::setInt1Duration(uint8_t duration) {
    lis2dh12_int1_gen_duration_set(&dev_ctx, duration);
}
uint8_t IIA::getInt1Duration(void) {
    uint8_t duration;
    lis2dh12_int1_gen_duration_get(&dev_ctx, &duration);
    return (duration);
}

void IIA::setIntPolarity(uint8_t level) {
    lis2dh12_ctrl_reg6_t val;
    lis2dh12_pin_int2_config_get(&dev_ctx, &val);

    if (level == HIGH)
        val.int_polarity = 0;  // Clear INT_POLARITY bit for active high
    else
        val.int_polarity = 1;  // Set INT_POLARITY bit for active low

    lis2dh12_pin_int2_config_set(&dev_ctx, &val);
}

void IIA::setInt1IA1(bool enable) {
    lis2dh12_ctrl_reg3_t val;
    lis2dh12_pin_int1_config_get(&dev_ctx, &val);

    if (enable == true)
        val.i1_ia1 = 1;  // Enable IA1 on INT1
    else
        val.i1_ia1 = 0;  // Disable IA1 on INT1

    lis2dh12_pin_int1_config_set(&dev_ctx, &val);
}

bool IIA::getInt1(void) {
    lis2dh12_int1_src_t val;
    lis2dh12_int1_gen_source_get(&dev_ctx, &val);

    if (val.ia)
        return (true);
    return (false);
}

void IIA::setInt1Latch(bool enable) {
    lis2dh12_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis2dh12_read_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
    if (ret == 0) {
        if (enable)
            ctrl_reg5.lir_int1 = 1;
        else
            ctrl_reg5.lir_int1 = 0;
        ret = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
    }
    // return ret;
}

// Enable X or Y as interrupt sources
void IIA::setInt1(bool enable) {
    lis2dh12_int1_cfg_t val;

    val.aoi = 0;  // Set 'Or' combination of interrupts
    val._6d = 0;  // Set 'Or' combination of interrupts
    if (enable)
        val.xhie = 1;
    else
        val.xhie = 0;

    val.xlie = 0;  // Do not set both low and high

    if (enable)
        val.yhie = 1;
    else
        val.yhie = 0;

    val.ylie = 0;
    val.zhie = 0;  // Leave out Z otherwise it will always trigger when sitting on table
    val.zlie = 0;
    lis2dh12_int1_gen_conf_set(&dev_ctx, &val);
}

// Enable single tap detection
void IIA::enableTapDetection() {
    lis2dh12_click_cfg_t newBits;
    if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0) {
        newBits.xs = true;
        newBits.ys = true;
        newBits.zs = true;
        lis2dh12_tap_conf_set(&dev_ctx, &newBits);
    }
}

// Disable single tap detection
void IIA::disableTapDetection() {
    lis2dh12_click_cfg_t newBits;
    if (lis2dh12_tap_conf_get(&dev_ctx, &newBits) == 0) {
        newBits.xs = false;
        newBits.ys = false;
        newBits.zs = false;
        lis2dh12_tap_conf_set(&dev_ctx, &newBits);
    }
}

// Set 7 bit threshold value
void IIA::setTapThreshold(uint8_t threshold) {
    if (threshold > 127)  // Register is 7 bits wide
        threshold = 127;
    lis2dh12_tap_threshold_set(&dev_ctx, threshold);
}

// Returns true if a tap is detected
bool IIA::isTapped(void) {
    lis2dh12_click_src_t interruptSource;
    lis2dh12_tap_source_get(&dev_ctx, &interruptSource);
    if (interruptSource.x || interruptSource.y || interruptSource.z)  // Check if ZYX bits are set
    {
        return (true);
    }
    return (false);
}

/*
   @brief  Write generic device register (platform dependent)

   @param  handle    customizable argument. In this examples is used in
                     order to select the correct sensor bus handler.
   @param  reg       register to write
   @param  bufp      pointer to data to write in register reg
   @param  len       number of consecutive register to write

*/
int32_t IIA::platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    IIA *classPointer = (IIA *)handle;

    if (len > 30) {
        return 1;  // Error
    }

    classPointer->_i2cPort->beginTransmission(classPointer->_i2cAddress);
    classPointer->_i2cPort->write(reg);
    for (uint16_t x = 0; x < len; x++) {
        classPointer->_i2cPort->write(bufp[x]);
    }

    byte endT = classPointer->_i2cPort->endTransmission();
    return (endT);  // Will return 0 upon success
}

/*
   @brief  Read generic device register (platform dependent)

   @param  handle    customizable argument. In this examples is used in
                     order to select the correct sensor bus handler.
   @param  reg       register to read
   @param  bufp      pointer to buffer that store the data read
   @param  len       number of consecutive register to read

*/
int32_t IIA::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    IIA *classPointer = (IIA *)handle;

    if (len > 1) {
        // For multi byte reads we must set the first bit to 1
        reg |= 0x80;
    }

    classPointer->_i2cPort->beginTransmission(classPointer->_i2cAddress);
    classPointer->_i2cPort->write(reg);
    classPointer->_i2cPort->endTransmission(false);  // Don't release bus. Will return 0 upon success.

    classPointer->_i2cPort->requestFrom((uint8_t)classPointer->_i2cAddress, (uint8_t)len);
    for (uint16_t x = 0; x < len; x++) {
        bufp[x] = classPointer->_i2cPort->read();
    }

    return (0);  // Success
}

/******************************************************************************
KXTJ3
******************************************************************************/

#include "stdint.h"

//****************************************************************************//
//
//  Default construction is I2C mode, address 0x0E.
//
//****************************************************************************//

//****************************************************************************//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//****************************************************************************//
IIA_status_t IIA::readRegisterRegion(uint8_t *outputPointer, uint8_t offset,
                                     uint8_t length) {
    IIA_status_t returnError = IMU_SUCCESS;

    // define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;

    Wire.beginTransmission(I2CAddress);
    offset |= 0x80;  // turn auto-increment bit on, bit 7 for I2C
    Wire.write(offset);
    if (Wire.endTransmission() != 0) {
        return IMU_HW_ERROR;
    } else  // OK, all worked, keep going
    {
        // request 6 bytes from slave device
        Wire.requestFrom(I2CAddress, length);
        while ((Wire.available()) &&
               (i < length))  // slave may send less than requested
        {
            c = Wire.read();  // receive a byte as character
            *outputPointer = c;
            outputPointer++;
            i++;
        }
    }

    return returnError;
}

//****************************************************************************//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//****************************************************************************//
IIA_status_t IIA::readRegister(uint8_t *outputPointer, uint8_t offset) {
    // Return value
    uint8_t result = 0;
    uint8_t numBytes = 1;
    IIA_status_t returnError = IMU_SUCCESS;

    Wire.beginTransmission(I2CAddress);
    Wire.write(offset);

    if (Wire.endTransmission() != 0) {
        return IMU_HW_ERROR;
    }

    Wire.requestFrom(I2CAddress, numBytes);

    while (Wire.available()) {
        result = Wire.read();  // receive a byte as a proper uint8_t
    }

    if (debugMode) {
        IIA_DEBUG.print("Read register 0x");
        IIA_DEBUG.print(offset, HEX);
        IIA_DEBUG.print(" = 0x");
        IIA_DEBUG.println(result, HEX);
    }

    *outputPointer = result;
    return returnError;
}

//****************************************************************************//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//****************************************************************************//
IIA_status_t IIA::readRegisterInt16(int16_t *outputPointer, uint8_t offset) {
    // offset |= 0x80; //turn auto-increment bit on
    uint8_t myBuffer[2];
    IIA_status_t returnError =
        readRegisterRegion(myBuffer, offset, 2);  // Does memory transfer
    int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

    if (debugMode && returnError == IMU_SUCCESS) {
        IIA_DEBUG.print("16 bits from 0x");
        IIA_DEBUG.print(offset, HEX);
        IIA_DEBUG.print(" = ");
        IIA_DEBUG.println(output);
    } else if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    *outputPointer = output;
    return returnError;
}

//****************************************************************************//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//****************************************************************************//
IIA_status_t IIA::writeRegister(uint8_t offset, uint8_t dataToWrite) {
    IIA_status_t returnError = IMU_SUCCESS;

    // Write the byte
    Wire.beginTransmission(I2CAddress);
    Wire.write(offset);
    Wire.write(dataToWrite);
    if (Wire.endTransmission() != 0) {
        returnError = IMU_HW_ERROR;
    }

    return returnError;
}

//****************************************************************************//
//
//  softwareReset
//  Resets the device; recommended by Kionix on initial power-up (TN017)
//
//****************************************************************************//
IIA_status_t IIA::softwareReset(void) {
    IIA_status_t returnError = IMU_SUCCESS;

    // Start by copying the current I2C address to a temp variable
    // We must do this because the IMU could boot with a bit-flipped address
    uint8_t tempAddress = I2CAddress;

    // Write 0x00 to IIA_SOFT_REST to confirm IMU is on the bus at address
    Wire.beginTransmission(I2CAddress);
    Wire.write(IIA_SOFT_REST);
    Wire.write(0x00);

    // If NACK returned, switch I2CAddress to flipped version and try again
    if (Wire.endTransmission() != 0) {
        if (I2CAddress == 0x0F) {
            I2CAddress = 0x0D;
        } else if (I2CAddress == 0x0E) {
            I2CAddress = 0x0C;
        }

        Wire.beginTransmission(I2CAddress);
        Wire.write(IIA_SOFT_REST);
        Wire.write(0x00);

        // If still NACK, give up, need to power cycle IMU to recover
        if (Wire.endTransmission() != 0) {
            // Return I2CAddress to normal before returning
            if (I2CAddress != tempAddress) {
                I2CAddress = tempAddress;
            }
            return IMU_HW_ERROR;
        }
    }

    // Attempt to address CTRL_REG2 and end if NACK returned
    Wire.beginTransmission(I2CAddress);
    Wire.write(IIA_CTRL_REG2);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0) {
        // Return I2CAddress to normal before returning
        if (I2CAddress != tempAddress) {
            I2CAddress = tempAddress;
        }
        return IMU_HW_ERROR;
    }

    // Send software reset command to CTRL_REG2 and end if NACK returned
    Wire.beginTransmission(I2CAddress);
    Wire.write(IIA_CTRL_REG2);
    Wire.write(0x80);

    if (Wire.endTransmission() != 0) {
        // Return I2CAddress to normal before returning
        if (I2CAddress != tempAddress) {
            I2CAddress = tempAddress;
        }
        return IMU_HW_ERROR;
    }

    // Set I2CAddress back to normal since we've successfully reset the IMU
    if (I2CAddress != tempAddress) {
        I2CAddress = tempAddress;
    }

    // Delay for software start-up before returning (TN017 Table 1)
    delay(2);

    return returnError;
}

//****************************************************************************//
//
//  Read axis acceleration as Float
//
//****************************************************************************//
float IIA::axisAccel(axis_t _axis) {
    int16_t outRAW;
    uint8_t regToRead = 0;
    switch (_axis) {
        case 0:
            // X axis
            regToRead = IIA_XOUT_L;
            break;
        case 1:
            // Y axis
            regToRead = IIA_YOUT_L;
            break;
        case 2:
            // Z axis
            regToRead = IIA_ZOUT_L;
            break;

        default:
            // Not valid axis return NAN
            return NAN;
            break;
    }

    // Don't proceed if the read failed
    if (readRegisterInt16(&outRAW, regToRead) != IMU_SUCCESS) {
        return NAN;
    }

    // The LSB may contain garbage, so 0 any unused bits
    if (!highRes) {
        outRAW &= 0b1111111100000000;  // 8-bit mode
    } else if (en14Bit) {
        outRAW &= 0b1111111111111100;  // 14-bit mode
    } else {
        outRAW &= 0b1111111111110000;  // 12-bit mode
    }

    float outFloat;

    switch (accelRange) {
        case 2:
            outFloat = (float)outRAW / 16384;
            break;
        case 4:
            outFloat = (float)outRAW / 8192;
            break;
        case 8:
            outFloat = (float)outRAW / 4096;
            break;
        case 16:
            outFloat = (float)outRAW / 2048;
            break;
        default:
            outFloat = 0;
            break;
    }

    return outFloat;
}

//****************************************************************************//
//
//  Place the accelerometer into/out of standby
//
//****************************************************************************//
IIA_status_t IIA::standby(bool _en) {
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t _ctrl;

    // "Backup" IIA_CTRL_REG1
    returnError = readRegister(&_ctrl, IIA_CTRL_REG1);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    if (_en)
        _ctrl &= 0x7E;
    else
        _ctrl |= (0x01 << 7);  // disable standby-mode -> Bit7 = 1 = operating mode

    returnError = writeRegister(IIA_CTRL_REG1, _ctrl);

    // If taking out of standby, follow start-up delay
    if (!_en && returnError == IMU_SUCCESS) {
        startupDelay();
    }

    return returnError;
}

//****************************************************************************//
//
//  Applies the start-up delay specified in Table 1 of the DataSheet
//  Used when coming out of standby or softwareReset
//
//****************************************************************************//
void IIA::startupDelay(void) {
    if (highRes) {
        if (accelSampleRate < 1)
            delay(1300);
        else if (accelSampleRate < 3)
            delay(650);
        else if (accelSampleRate < 6)
            delay(330);
        else if (accelSampleRate < 12)
            delay(170);
        else if (accelSampleRate < 25)
            delay(90);
        else if (accelSampleRate < 50)
            delay(45);
        else if (accelSampleRate < 100)
            delay(25);
        else if (accelSampleRate < 200)
            delay(11);
        else if (accelSampleRate < 400)
            delay(6);
        else if (accelSampleRate < 800)
            delay(4);
        else if (accelSampleRate < 1600)
            delay(3);
        else
            delay(2);
    } else {
        if (accelSampleRate < 800 && accelSampleRate > 200)
            delay(4);
        else if (accelSampleRate < 1600 && accelSampleRate > 400)
            delay(3);
        else
            delay(2);
    }
}

//****************************************************************************//
//
//  Apply settings passed to .begin();
//
//****************************************************************************//
IIA_status_t IIA::applySettings(void) {
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t dataToWrite = 0;  // Temporary variable

    standby(true);

    // Build DATA_CTRL_REG

    //  Convert ODR
    if (accelSampleRate < 1)
        dataToWrite |= 0x08;  // 0.781Hz
    else if (accelSampleRate < 2)
        dataToWrite |= 0x09;  // 1.563Hz
    else if (accelSampleRate < 4)
        dataToWrite |= 0x0A;  // 3.125Hz
    else if (accelSampleRate < 8)
        dataToWrite |= 0x0B;  // 6.25Hz
    else if (accelSampleRate < 16)
        dataToWrite |= 0x00;  // 12.5Hz
    else if (accelSampleRate < 30)
        dataToWrite |= 0x01;  // 25Hz
    else if (accelSampleRate < 60)
        dataToWrite |= 0x02;  // 50Hz
    else if (accelSampleRate < 150)
        dataToWrite |= 0x03;  // 100Hz
    else if (accelSampleRate < 250)
        dataToWrite |= 0x04;  // 200Hz
    else if (accelSampleRate < 450)
        dataToWrite |= 0x05;  // 400Hz
    else if (accelSampleRate < 850)
        dataToWrite |= 0x06;  // 800Hz
    else
        dataToWrite |= 0x07;  // 1600Hz

    // Now, write the patched together data
    if (debugMode) {
        IIA_DEBUG.print("IIA_DATA_CTRL_REG: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }
    returnError = writeRegister(IIA_DATA_CTRL_REG, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Build CTRL_REG1

    // LOW power, 8-bit mode
    dataToWrite = 0x80;

    if (highRes) {
        if (debugMode) {
            IIA_DEBUG.println("High Resolution set");
        }
        dataToWrite = 0xC0;
    }

    //  Convert scaling
    switch (accelRange) {
        default:
        case 2:
            dataToWrite |= (0x00 << 2);
            break;
        case 4:
            dataToWrite |= (0x02 << 2);
            break;
        case 8:
            dataToWrite |= (0x04 << 2);
            break;
        case 16:
            dataToWrite |= (0x01 << 2);
            break;
    }

    // Now, write the patched together data
    if (debugMode) {
        IIA_DEBUG.print("IIA_CTRL_REG1: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }
    returnError = writeRegister(IIA_CTRL_REG1, dataToWrite);
    startupDelay();
    return returnError;
}

//****************************************************************************//
//
//  Enables 14-bit operation mode for 8g/16g acceleration ranges
//
//****************************************************************************//
IIA_status_t IIA::enable14Bit(uint8_t accRange) {
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t dataToWrite = 0;  // Temporary variable
    accelRange = accRange;    // Set accelRange to new value
    highRes = true;           // Make sure highRes is set to true
    en14Bit = true;           // Set 14-bit check to true as well

    standby(true);

    if (debugMode) {
        IIA_DEBUG.println("Switching to 14-bit mode");
    }

    // Build CTRL_REG1

    switch (accelRange) {
        default:
        case 16:
            dataToWrite = 0xDC;
            break;
        case 8:
            dataToWrite = 0xD8;
            break;
    }

    // Write the new data to CTRL_REG1
    if (debugMode) {
        IIA_DEBUG.print("IIA_CTRL_REG1: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }
    returnError = writeRegister(IIA_CTRL_REG1, dataToWrite);

    if (returnError == IMU_SUCCESS) {
        startupDelay();
    }

    return returnError;
}

//****************************************************************************//
//  Configure interrupt, stop or move, threshold and duration
//	Duration, steps and maximum values depend on the ODR chosen.
//****************************************************************************//
IIA_status_t IIA::intConf(int16_t threshold, uint8_t moveDur,
                          uint8_t naDur, bool polarity, float wuRate,
                          bool latched, bool pulsed, bool motion,
                          bool dataReady, bool intPin) {
    IIA_status_t returnError = IMU_SUCCESS;

    // Note that to properly change the value of this register, the PC1 bit in
    // CTRL_REG1 must first be set to “0”.
    returnError = standby(true);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Build INT_CTRL_REG1

    uint8_t dataToWrite = 0x00;  // Interrupt pin disabled, active LOW, latched

    // uint8_t dataToWrite = 0x20; // Interrupt enabled, active LOW, latched

    if (pulsed)
        dataToWrite |= (0x01 << 3);  // Interrupt pin pulsed

    if (polarity == HIGH)
        dataToWrite |= (0x01 << 4);  // Active HIGH

    if (intPin)
        dataToWrite |= (0x01 << 5);  // Interrupt pin enabled

    if (debugMode) {
        IIA_DEBUG.print("IIA_INT_CTRL_REG1: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_INT_CTRL_REG1, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    uint8_t _reg1;

    // First "back up" current settings to a temporary variable
    returnError = readRegister(&_reg1, IIA_CTRL_REG1);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    if (motion)
        _reg1 |= (0x01 << 1);  // Sets WUFE to enabled

    if (dataReady)
        _reg1 |= (0x01 << 5);  // Sets DRDY to enabled

    if (debugMode) {
        IIA_DEBUG.print("IIA_CTRL_REG1: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_CTRL_REG1, _reg1);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Set data rate for Wake-Up Function

    if (wuRate < 0) {
        // Sets the Data Rate for the Wake-Up (motion detect) function to match ODR
        // Start by checking DATA_CTRL_REG for the current ODR

        returnError = readRegister(&_reg1, IIA_DATA_CTRL_REG);

        if (returnError != IMU_SUCCESS) {
            return returnError;
        }

        // Set ODRWU based on ODR
        // Maximum ODRWU is 100 Hz

        switch (_reg1) {
            case 0x09:
                dataToWrite = 0x01;  // 1.563 Hz
                break;
            case 0x0A:
                dataToWrite = 0x02;  // 3.125 Hz
                break;
            case 0x0B:
                dataToWrite = 0x03;  // 6.25 Hz
                break;
            case 0x00:
                dataToWrite = 0x04;  // 12.5 Hz
                break;
            case 0x01:
                dataToWrite = 0x05;  // 25 Hz
                break;
            case 0x02:
                dataToWrite = 0x06;  // 50 Hz
                break;
            case 0x03:
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x07:
                dataToWrite = 0x07;  // 100 Hz
                break;
            default:
                dataToWrite = 0x00;  // 0.781 Hz
                break;
        }
    } else if (wuRate < 1)
        dataToWrite = 0x00;  // 0x781 Hz
    else if (wuRate < 2)
        dataToWrite = 0x01;  // 1.563 Hz
    else if (wuRate < 4)
        dataToWrite = 0x02;  // 3.125 Hz
    else if (wuRate < 7)
        dataToWrite = 0x03;  // 6.25 Hz
    else if (wuRate < 13)
        dataToWrite = 0x04;  // 12.5 Hz
    else if (wuRate < 26)
        dataToWrite = 0x05;  // 25 Hz
    else if (wuRate < 51)
        dataToWrite = 0x06;  // 50 Hz
    else
        dataToWrite = 0x07;  // 100 Hz

    if (debugMode) {
        IIA_DEBUG.print("IIA_CTRL_REG2: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_CTRL_REG2, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Build INT_CTRL_REG2

    dataToWrite = 0x3F;  // enable interrupt on all axes any direction, latched

    if (!latched)
        dataToWrite |= (0x01 << 7);  // enable unlatched mode

    if (debugMode) {
        IIA_DEBUG.print("IIA_INT_CTRL_REG2: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_INT_CTRL_REG2, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Set WAKE-UP (motion detect) Threshold

    dataToWrite = (uint8_t)(threshold >> 4);

    if (debugMode) {
        IIA_DEBUG.print("IIA_WAKEUP_THRESHOLD_H: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_WAKEUP_THRESHOLD_H, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    dataToWrite = (uint8_t)(threshold << 4);

    if (debugMode) {
        IIA_DEBUG.print("IIA_WAKEUP_THRESHOLD_L: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_WAKEUP_THRESHOLD_L, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // WAKEUP_COUNTER -> Sets the time motion must be present before a wake-up
    // interrupt is set WAKEUP_COUNTER (counts) = Wake-Up Delay Time (sec) x
    // Wake-Up Function ODR(Hz)

    dataToWrite = moveDur;

    if (debugMode) {
        IIA_DEBUG.print("IIA_WAKEUP_COUNTER: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_WAKEUP_COUNTER, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Non-Activity register sets the non-activity time required before another
    // wake-up interrupt will be reported. NA_COUNTER (counts) = Non-ActivityTime
    // (sec) x Wake-Up Function ODR(Hz)

    dataToWrite = naDur;

    if (debugMode) {
        IIA_DEBUG.print("IIA_NA_COUNTER: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_NA_COUNTER, dataToWrite);

    if (returnError != IMU_SUCCESS) {
        return returnError;
    }

    // Set IMU to Operational mode
    returnError = standby(false);

    return returnError;
}

IIA_status_t IIA::intDisableAxis(uint8_t first) {
    // Create temporary variables
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t dataToWrite = 0b00111111;
    uint8_t bitCheck;

    // Check to see if ULMODE bit is set and set if so
    returnError = readRegister(&bitCheck, IIA_INT_CTRL_REG2);
    if (bitCheck & (0x01 << 7))
        dataToWrite |= (0x01 << 7);

    // Rebuild INT_CTRL_REG2 with new axis data using XOR
    dataToWrite ^= first;

    // Write the new values to INT_CTRL_REG2
    if (debugMode) {
        IIA_DEBUG.print("IIA_INT_CTRL_REG2: 0x");
        IIA_DEBUG.println(dataToWrite, HEX);
    }

    returnError = writeRegister(IIA_INT_CTRL_REG2, dataToWrite);

    return returnError;
}

IIA_status_t IIA::intDisableAxis(uint8_t first, uint8_t second) {
    // Create temporary variables
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t temp = 0x00;
    if (first == NONE || second == NONE) {
        returnError = intDisableAxis(temp);  // send 0x00 to enable all axes
    } else {
        // combine the requested axes and submit to base function
        temp |= first;
        temp |= second;
        returnError = intDisableAxis(temp);
    }

    return returnError;
}

IIA_status_t IIA::intDisableAxis(uint8_t first, uint8_t second,
                                 uint8_t third) {
    // Create temporary variables
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t temp = 0x00;
    if (first == NONE || second == NONE || third == NONE) {
        returnError = intDisableAxis(temp);  // send 0x00 to enable all axes
    } else {
        // combine the requested axes and submit to base function
        temp |= first;
        temp |= second;
        temp |= third;
        returnError = intDisableAxis(temp);
    }

    return returnError;
}

IIA_status_t IIA::intDisableAxis(uint8_t first, uint8_t second,
                                 uint8_t third, uint8_t fourth) {
    // Create temporary variables
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t temp = 0x00;
    if (first == NONE || second == NONE || third == NONE || fourth == NONE) {
        returnError = intDisableAxis(temp);  // send 0x00 to enable all axes
    } else {
        // combine the requested axes and submit to base function
        temp |= first;
        temp |= second;
        temp |= third;
        temp |= fourth;
        returnError = intDisableAxis(temp);
    }

    return returnError;
}

IIA_status_t IIA::intDisableAxis(uint8_t first, uint8_t second,
                                 uint8_t third, uint8_t fourth,
                                 uint8_t fifth) {
    // Create temporary variables
    IIA_status_t returnError = IMU_SUCCESS;
    uint8_t temp = 0x00;
    if (first == NONE || second == NONE || third == NONE || fourth == NONE ||
        fifth == NONE) {
        returnError = intDisableAxis(temp);  // send 0x00 to enable all axes
    } else {
        // combine the requested axes and submit to base function
        temp |= first;
        temp |= second;
        temp |= third;
        temp |= fourth;
        temp |= fifth;
        returnError = intDisableAxis(temp);
    }

    return returnError;
}

bool IIA::dataReady(void) {
    uint8_t _reg1;

    readRegister(&_reg1, IIA_INT_SOURCE1);

    // Bit 4 is Data Ready Interrupt Bit
    if (_reg1 & (0x01 << 4)) {
        return true;
    } else {
        return false;
    }
}

bool IIA::motionDetected(void) {
    uint8_t _reg1;

    readRegister(&_reg1, IIA_INT_SOURCE1);

    // Bit 1 is Wake-Up Function Sense Bit
    if (_reg1 & (0x01 << 1)) {
        return true;
    } else {
        return false;
    }
}

wu_axis_t IIA::motionDirection(void) {
    uint8_t _reg1;

    readRegister(&_reg1, IIA_INT_SOURCE2);

    if (_reg1 & (0x01 << 0))
        return ZPOS;
    else if (_reg1 & (0x01 << 1))
        return ZNEG;
    else if (_reg1 & (0x01 << 2))
        return YPOS;
    else if (_reg1 & (0x01 << 3))
        return YNEG;
    else if (_reg1 & (0x01 << 4))
        return XPOS;
    else if (_reg1 & (0x01 << 5))
        return XNEG;
    else
        return NONE;
}

IIA_status_t IIA::resetInterrupt(void) {
    IIA_status_t returnError = IMU_SUCCESS;

    uint8_t _reg1;

    // Reading the INT_REL register releases the latch
    returnError = readRegister(&_reg1, IIA_INT_REL);

    return returnError;
}