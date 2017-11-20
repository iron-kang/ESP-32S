#ifndef __BAT_MAX17043__
#define __BAT_MAX17043__

#include "stdint.h"
#include "stdbool.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define MAX17043_ADDRESS                0x36
//02h-03h.  Reports 12-bit A/D measurement of battery voltage.  READ-ONLY.
#define MAX17043_VCELL                  0x02
// vcell reports battery in 0.00125V (1/800V or 1.25mV) increments
#define MAX17043_VCELL_VOLTAGE_FACTOR	0.00125
//04h-05h.  Reports 16-bit SOC result calculated by ModelGauge algorithm.  READ-ONLY.
#define MAX17043_SOC                    0x04
//06h-07h.  Sends special commands to the IC.  WRITE-ONLY
#define MAX17043_MODE                   0x06
//08h-09h.  Returns IC version.  READ-ONLY.
#define MAX17043_VERSION                0x08
//0Ch-0Dh.  Battery compensation. Adjusts IC performance based on application conditions.
//READ & WRITE.  Default:  971Ch.
#define MAX17043_CONFIG                 0x0C
/* CONFIG */
#define MAX17043_AWAKE_DEFAULT          0x9700 //lsb is 00000000
#define MAX17043_SLEEP_DEFAULT          0x9780 //lsb is 10000000
#define MAX17043_IS_TRUE       	        0x01
#define MAX17043_MAX_ALERT_PERCENTAGE   32
#define MAX17043_MIN_ALERT_PERCENTAGE   1
//FEh-FFh.  Sends special commands to the IC.  WRITE-ONLY.
#define MAX17043_COMMAND                0xFE
/* COMMAND */
#define MAX17043_POWER_ON_RESET         0x5400
#define MAX17043_QUICK_START            0x4000

#define I2C_NUM							I2C_NUM_1
#define ACK_VAL    						0x0
#define NACK_VAL   						0x1
#define ACK_CHECK_EN   					0x1

typedef struct BatteryInfo{
    uint8_t SDAPin;
    uint8_t SCLPin;
    uint8_t alertPin;
    uint8_t alertThreshold;
    bool sleeping;

    esp_err_t (*begin)(struct BatteryInfo *self);
    esp_err_t (*reset)(struct BatteryInfo *self);
    
    /* 
    quickStart() issues a quick-start command to the MAX170433.
    A quick start allows the MAX170433 to restart fuel-gauge calculations
    in the same manner as initial power-up of the IC. If an application's
    power-up sequence is very noisy, such that excess error is introduced
    into the IC's first guess of SOC, the Arduino can issue a quick-start
    to reduce the error.
    */
    esp_err_t (*quickStart)(struct BatteryInfo *self);
    uint16_t (*getVersion)(struct BatteryInfo *self);

    /*
    batteryVoltage() returns a float value voltage based on the 12-bit ADC reading
    of the battery as reported by the MAX170433's VCELL register
    */
    float (*getBatteryVoltage)(struct BatteryInfo *self);

    /*
    batteryPercentage() returns a float value of the battery percentage
    reported from the SOC register of the MAX170433.
    */
    float (*getBatteryPercentage)(struct BatteryInfo *self);

    /* 
    setAlertThreshold(uint8_t percent) configures the config register of
    the MAX170143, specifically the alert threshold therein. Pass a 
    value between 1 and 32 to set the alert threshold to a value between
    1 and 32%. Anything below will be set to 1%, above 32%.
    */
    void (*setAlertThreshold)(struct BatteryInfo *self, uint8_t percent);
    uint8_t (*getAlertThreshold)(struct BatteryInfo *self);
    //gets it from the register
    uint8_t (*getAlertThresholdRegister)(struct BatteryInfo *self);
    bool (*isAlerting)(struct BatteryInfo *self);
    void (*clearAlert)(struct BatteryInfo *self);
  	
    //gets it from the register
    bool (*isSleepingRegister)(struct BatteryInfo *self);
    void (*sleep)(struct BatteryInfo *self);
    void (*wake)(struct BatteryInfo *self);

} MAX17043;

void MAX17043_Init(MAX17043 *fuel_gauge);

#endif
