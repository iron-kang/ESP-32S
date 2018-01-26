#ifndef HMC5883L_h
#define HMC5883L_h

#include "esp_err.h"
#include "esp32_i2c.h"

#define HMC5883L_ADDRESS              (0x1E)
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

typedef enum
{
    HMC5883L_SAMPLES_8     = 0b11,
    HMC5883L_SAMPLES_4     = 0b10,
    HMC5883L_SAMPLES_2     = 0b01,
    HMC5883L_SAMPLES_1     = 0b00
} hmc5883l_samples_t;

typedef enum
{
    HMC5883L_DATARATE_75HZ       = 0b110,
    HMC5883L_DATARATE_30HZ       = 0b101,
    HMC5883L_DATARATE_15HZ       = 0b100,
    HMC5883L_DATARATE_7_5HZ      = 0b011,
    HMC5883L_DATARATE_3HZ        = 0b010,
    HMC5883L_DATARATE_1_5HZ      = 0b001,
    HMC5883L_DATARATE_0_75_HZ    = 0b000
} hmc5883l_dataRate_t;

typedef enum
{
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000
} hmc5883l_range_t;

typedef enum
{
    HMC5883L_IDLE          = 0b10,
    HMC5883L_SINGLE        = 0b01,
    HMC5883L_CONTINOUS     = 0b00
} hmc5883l_mode_t;

typedef struct vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
}Vector;

typedef struct hmc
{
    I2C_CONFIG i2c;
    
    uint8_t SDAPin;
    uint8_t SCLPin;
    float mgPerDigit;
    int xOffset, yOffset;
    Vector v;

    Vector (*readRaw)(struct hmc *self);
    Vector (*readNormalize)(struct hmc *self);
    void (*setOffset)(struct hmc *self, int xo, int yo);
    void (*setRange)(struct hmc *self, hmc5883l_range_t range);
    hmc5883l_range_t (*getRange)(struct hmc *self);
    void (*setMeasurementMode)(struct hmc *self, hmc5883l_mode_t mode);
    hmc5883l_mode_t (*getMeasurementMode)(struct hmc *self);
    void (*setDataRate)(struct hmc *self, hmc5883l_dataRate_t dataRate);
    hmc5883l_dataRate_t (*getDataRate)(struct hmc *self);
    void (*setSamples)(struct hmc *self, hmc5883l_samples_t samples);
    hmc5883l_samples_t (*getSamples)(struct hmc *self);


}HMC5883L;

esp_err_t HMC5883L_Init(HMC5883L *hmc);

#endif
