#ifndef IIA_H
#define IIA_H
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "lis2dh12_reg.h"  //This is the ST library
#define LIS2DH12_ADDR 0x19
#define KXTJ3_ADDR 0x0F
enum IIA_RANGE {
    RANGE_2G = 0,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G
};

// Print variable name
#define getName(var) #var

// Return values
typedef enum {
    IMU_SUCCESS,
    IMU_HW_ERROR,
    IMU_NOT_SUPPORTED,
    IMU_GENERIC_ERROR,
    IMU_OUT_OF_BOUNDS,
    IMU_ALL_ONES_WARNING,
    //...
} IIA_status_t;

typedef enum {
    Xacc = 0,
    Yacc,
    Zacc,
} axis_t;

typedef enum {
    NONE = 0,
    ZPOS = 1,
    ZNEG = 2,
    YPOS = 4,
    YNEG = 8,
    XPOS = 16,
    XNEG = 32,
} wu_axis_t;

class IIA {
   public:
    // LIS2DH12
    IIA();
    bool getData(float &x, float &y, float &z);
    bool getJSON(JsonDocument &doc);

    bool begin(uint8_t range = RANGE_2G, float sampleRate = 6.25);  // Begin comm with accel at given I2C address, and given wire port
    bool isConnected();                                                                                                            // Returns true if an accel sensor is detected at library's I2C address
    bool available();                                                                                                              // Returns true if new accel data is available
    void waitForNewData();                                                                                                         // Block until new data is available
    bool temperatureAvailable();                                                                                                   // Returns true if new temp data is available

    float getX();  // Return latest accel data in milli-g's. If data has already be read, initiate new read.
    float getY();
    float getZ();
    int16_t getRawX();  // Return raw 16 bit accel reading
    int16_t getRawY();
    int16_t getRawZ();
    float getTemperature();  // Returns latest temp data in C. If data is old, initiate new read.

    void parseAccelData();  // Load sensor data into global vars. Call after new data is avaiable.
    void getTempData();

    void enableTemperature();   // Enable the onboard temp sensor
    void disableTemperature();  // Disable the onboard temp sensor

    void setDataRate(uint8_t dataRate);  // Set the output data rate of sensor. Higher rates consume more current.
    uint8_t getDataRate();               // Returns the output data rate of sensor.

    void setScale(uint8_t scale);  // Set full scale: +/-2, 4, 8, or 16g
    uint8_t getScale();            // Returns current scale of sensor

    void setMode(uint8_t mode);  // Set mode to low, normal, or high data rate
    uint8_t getMode();           // Get current sensor mode

    void enableSelfTest(bool direction = true);
    void disableSelfTest();

    void enableTapDetection();  // Enable the single tap interrupt
    void disableTapDetection();
    void setTapThreshold(uint8_t threshold);  // Set the 7-bit threshold value for tap and double tap
    bool isTapped();                          // Returns true if Z, Y, or X tap detection bits are set

    void setInt1Threshold(uint8_t threshold);
    uint8_t getInt1Threshold(void);
    void setInt1Duration(uint8_t duration);
    uint8_t getInt1Duration(void);

    void setIntPolarity(uint8_t level);
    void setInt1IA1(bool enable);
    void setInt1Latch(bool enable);
    void setInt1(bool enable);

    bool getInt1(void);

    lis2dh12_ctx_t dev_ctx;

    static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
    static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

    uint8_t _i2cAddress;
    TwoWire *_i2cPort;

    // IIA

    /*
  Accelerometer range = 2, 4, 8, 16g
  Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800,
  1600Hz Output Data Rates ≥400Hz will force device into High Resolution mode
  */
   //  IIA_status_t begin(float sampleRate, uint8_t accRange,
   //                       bool highResSet = false, bool debugSet = false);

    // Enables 14-bit operation mode for Accelerometer range 8g/16g
    IIA_status_t enable14Bit(uint8_t accRange);

    // readRegister reads one 8-bit register
    IIA_status_t readRegister(uint8_t *outputPointer, uint8_t offset);

    // Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
    // Acts as a 16-bit read operation
    IIA_status_t readRegisterInt16(int16_t *outputPointer, uint8_t offset);

    // Writes an 8-bit byte;
    IIA_status_t writeRegister(uint8_t offset, uint8_t dataToWrite);

    // Configure Interrupts
    // @Threshold from -2048 to 2047 counts
    // @moveDur   from 1 to 255 counts
    // @naDur			from 1 to 255 counts
    // @polarity changes active low/high of physical interrupt pin
    // @wuRate    from 0.781 to 100Hz; -1 uses IMU data rate instead
    // @latched sets whether to use latched or unlatched interrupt
    // @pulsed sets whether to pulse the interrupt pin when active
    // @motion sets whether to trigger interrupt with wake-up function
    // @dataReady sets whether to trigger interrupt when new data is ready
    // @intPin sets whether to enable the interrupt pin or not
    // Threshold (g) = threshold (counts) / 256(counts/g)
    // timeDur (sec) = WAKEUP_COUNTER (counts) / Wake-Up Function ODR(Hz)
    // Non-ActivityTime (sec) = NA_COUNTER (counts) / Wake-Up Function ODR(Hz)
    IIA_status_t intConf(int16_t threshold, uint8_t moveDur, uint8_t naDur,
                           bool polarity = HIGH, float wuRate = -1,
                           bool latched = false, bool pulsed = false,
                           bool motion = true, bool dataReady = false,
                           bool intPin = true);

    IIA_status_t intDisableAxis(uint8_t first);
    IIA_status_t intDisableAxis(uint8_t first, uint8_t second);
    IIA_status_t intDisableAxis(uint8_t first, uint8_t second, uint8_t third);
    IIA_status_t intDisableAxis(uint8_t first, uint8_t second, uint8_t third,
                                  uint8_t fourth);
    IIA_status_t intDisableAxis(uint8_t first, uint8_t second, uint8_t third,
                                  uint8_t fourth, uint8_t fifth);

    // Checks to see if new data is ready (only works if DRDY interrupt enabled)
    bool dataReady(void);

    // Checks if the reason for the interrupt is the Wake-Up Function if enabled
    bool motionDetected(void);

    // Returns the direction that caused the Wake-Up Function to trigger
    wu_axis_t motionDirection(void);

    // Resets the interrupt latch
    IIA_status_t resetInterrupt(void);

    // Read axis acceleration as Float
    float axisAccel(axis_t _axis);

    // Set IMU to Standby ~0.9uA, also Enable configuration -> PC1 bit in
    // CTRL_REG1 must first be set to “0”
    IIA_status_t standby(bool _en = true);

   private:
    // LIS2DH12
    bool beginLIS2DH12(uint8_t range, uint8_t i2cAddress);

    int whichIMU();
    bool ping(int8_t addr);

    bool xIsFresh = false;
    bool yIsFresh = false;
    bool zIsFresh = false;
    bool tempIsFresh = false;

    uint8_t currentScale = 0;  // Needed to convert readings to mg
    uint8_t currentMode = 0;   // Needed to convert readings to mg

    float accelX;
    float accelY;
    float accelZ;
    uint16_t rawX;
    uint16_t rawY;
    uint16_t rawZ;
    float temperatureC;

    // KXTJ3
    bool beginKXTJ3(uint8_t range, float sampleRate, uint8_t i2cAddress);

    bool highRes = false;
    bool debugMode = false;
    bool en14Bit = false;
    uint8_t I2CAddress;
    float accelSampleRate;  // Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25,
                            // 50, 100, 200, 400, 800, 1600Hz
    uint8_t accelRange;     // Accelerometer range = 2, 4, 8, 16g

    // Apply settings at .begin()
    IIA_status_t applySettings(void);

    // Performs software reset
    IIA_status_t softwareReset(void);

    // ReadRegisterRegion takes a uint8 array address as input and reads
    //   a chunk of memory into that array.
    IIA_status_t readRegisterRegion(uint8_t *, uint8_t, uint8_t);

    // Start-up delay for coming out of standby
    void startupDelay(void);
};

// Device Registers
#define IIA_WHO_AM_I 0x0F
#define IIA_DCST_RESP \
    0x0C                      // used to verify proper integrated circuit functionality.
                              // It always has a byte value of 0x55
#define IIA_SOFT_REST 0x7F  // used during software reset
#define IIA_XOUT_L 0x06
#define IIA_XOUT_H 0x07
#define IIA_YOUT_L 0x08
#define IIA_YOUT_H 0x09
#define IIA_ZOUT_L 0x0A
#define IIA_ZOUT_H 0x0B

#define IIA_STATUS_REG 0x18
#define IIA_INT_SOURCE1 0x16
#define IIA_INT_SOURCE2 0x17
#define IIA_INT_REL 0x1A

#define IIA_CTRL_REG1 0x1B  // *
#define IIA_CTRL_REG2 0x1D  // *

#define IIA_INT_CTRL_REG1 0x1E  // *
#define IIA_INT_CTRL_REG2 0x1F  // *

#define IIA_DATA_CTRL_REG 0x21   // *
#define IIA_WAKEUP_COUNTER 0x29  // *
#define IIA_NA_COUNTER 0x2A      // *
#define IIA_SELF_TEST 0x3A       // *

#define IIA_WAKEUP_THRESHOLD_H 0x6A  // *
#define IIA_WAKEUP_THRESHOLD_L 0x6B  // *

#endif  // IIA_H