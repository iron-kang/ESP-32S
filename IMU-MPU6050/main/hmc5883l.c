#include "common.h"
#include "hmc5883l.h"

Vector _readRaw(HMC5883L *self);
Vector _readNormalize(HMC5883L *self);
void _checkSettings(HMC5883L *self);
void _setOffset(HMC5883L *self, int xo, int yo);
void _setRange(HMC5883L *self, hmc5883l_range_t range);
void _setMeasurementMode(HMC5883L *self, hmc5883l_mode_t mode);
void _setDataRate(HMC5883L *self, hmc5883l_dataRate_t dataRate);
void _setSamples(HMC5883L *self, hmc5883l_samples_t samples);
hmc5883l_range_t _getRange(HMC5883L *self);
hmc5883l_mode_t _getMeasurementMode(HMC5883L *self);
hmc5883l_dataRate_t _getDataRate(HMC5883L *self);
hmc5883l_samples_t _getSamples(HMC5883L *self);

int16_t readRegister16(I2C_CONFIG *i2c, uint8_t reg);
uint8_t readRegister8(I2C_CONFIG *i2c, uint8_t reg);
void writeRegister8(I2C_CONFIG *i2c, uint8_t reg, uint8_t value);

esp_err_t HMC5883L_Init(HMC5883L *hmc)
{
    
    if ((readRegister8(hmc->i2c, HMC5883L_REG_IDENT_A) != 0x48)
    ||  (readRegister8(hmc->i2c, HMC5883L_REG_IDENT_B) != 0x34)
    ||  (readRegister8(hmc->i2c, HMC5883L_REG_IDENT_C) != 0x33))
    {
        printf("HMC fail\n");
	return ESP_FAIL;
    }

    hmc->readRaw = _readRaw;
    hmc->readNormalize = _readNormalize;
    hmc->setOffset = _setOffset;
    hmc->setRange = _setRange;
    hmc->getRange = _getRange;
    hmc->setMeasurementMode = _setMeasurementMode;
    hmc->setDataRate = _setDataRate;
    hmc->setSamples = _setSamples;
    hmc->getMeasurementMode = _getMeasurementMode;
    hmc->getDataRate = _getDataRate;
    hmc->getSamples = _getSamples;
    hmc->checkSettings = _checkSettings;

    hmc->setRange(hmc, HMC5883L_RANGE_1_3GA);
    hmc->setMeasurementMode(hmc, HMC5883L_CONTINOUS);
    hmc->setDataRate(hmc, HMC5883L_DATARATE_30HZ);
    hmc->setSamples(hmc, HMC5883L_SAMPLES_8);
    hmc->setOffset(hmc, 0, 0); 
    hmc->mgPerDigit = 0.92f;

    return ESP_OK;
}

Vector _readRaw(HMC5883L *self)
{
    self->v.x = readRegister16(self->i2c, HMC5883L_REG_OUT_X_M) - self->xOffset;
    self->v.y = readRegister16(self->i2c, HMC5883L_REG_OUT_Y_M) - self->yOffset;
    self->v.z = readRegister16(self->i2c, HMC5883L_REG_OUT_Z_M);

    return self->v;
}

Vector _readNormalize(HMC5883L *self)
{
    self->v.x = ((float)readRegister16(self->i2c, HMC5883L_REG_OUT_X_M) - self->xOffset) * self->mgPerDigit;
    self->v.y = ((float)readRegister16(self->i2c, HMC5883L_REG_OUT_Y_M) - self->yOffset) * self->mgPerDigit;
    self->v.z = (float)readRegister16(self->i2c, HMC5883L_REG_OUT_Z_M) * self->mgPerDigit;

    return self->v;
}

void _setOffset(HMC5883L *self, int xo, int yo)
{
    self->xOffset = xo;
    self->yOffset = yo;
}

void _setRange(HMC5883L *self, hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    self->mgPerDigit = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    self->mgPerDigit = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    self->mgPerDigit = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    self->mgPerDigit = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    self->mgPerDigit = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    self->mgPerDigit = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    self->mgPerDigit = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    self->mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(self->i2c, HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t _getRange(HMC5883L *self)
{
    return (hmc5883l_range_t)((readRegister8(self->i2c, HMC5883L_REG_CONFIG_B) >> 5));
}

void _setMeasurementMode(HMC5883L *self, hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(self->i2c, HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t _getMeasurementMode(HMC5883L *self)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void _setDataRate(HMC5883L *self, hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(self->i2c, HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t _getDataRate(HMC5883L *self)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void _setSamples(HMC5883L *self, hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(self->i2c, HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t _getSamples(HMC5883L *self)
{
    uint8_t value;

    value = readRegister8(self->i2c, HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

void _checkSettings(HMC5883L *self)
{
    printf("Selected range: ");

    switch (self->getRange(self))
    {
        case HMC5883L_RANGE_0_88GA: printf("0.88 Ga\n"); break;
        case HMC5883L_RANGE_1_3GA:  printf("1.3 Ga\n"); break;
        case HMC5883L_RANGE_1_9GA:  printf("1.9 Ga\n"); break;
        case HMC5883L_RANGE_2_5GA:  printf("2.5 Ga\n"); break;
        case HMC5883L_RANGE_4GA:    printf("4 Ga\n"); break;
        case HMC5883L_RANGE_4_7GA:  printf("4.7 Ga\n"); break;
        case HMC5883L_RANGE_5_6GA:  printf("5.6 Ga\n"); break;
        case HMC5883L_RANGE_8_1GA:  printf("8.1 Ga\n"); break;
        default: printf("Bad range!\n");
    }

    printf("Selected Measurement Mode: ");
    switch (self->getMeasurementMode(self))
    {  
        case HMC5883L_IDLE: printf("Idle mode\n"); break;
        case HMC5883L_SINGLE:  printf("Single-Measurement\n"); break;
        case HMC5883L_CONTINOUS:  printf("Continuous-Measurement\n"); break;
        default: printf("Bad mode!\n");
    }

    printf("Selected Data Rate: ");
    switch (self->getDataRate(self))
    {  
        case HMC5883L_DATARATE_0_75_HZ: printf("0.75 Hz\n"); break;
        case HMC5883L_DATARATE_1_5HZ:   printf("1.5 Hz\n"); break;
        case HMC5883L_DATARATE_3HZ:     printf("3 Hz\n"); break;
        case HMC5883L_DATARATE_7_5HZ:   printf("7.5 Hz\n"); break;
        case HMC5883L_DATARATE_15HZ:    printf("15 Hz\n"); break;
        case HMC5883L_DATARATE_30HZ:    printf("30 Hz\n"); break;
        case HMC5883L_DATARATE_75HZ:    printf("75 Hz\n"); break;
        default: printf("Bad data rate!\n");
    }

    printf("Selected number of samples: ");
    switch (self->getSamples(self))
    {  
        case HMC5883L_SAMPLES_1: printf("1\n"); break;
        case HMC5883L_SAMPLES_2: printf("2\n"); break;
        case HMC5883L_SAMPLES_4: printf("4\n"); break;
        case HMC5883L_SAMPLES_8: printf("8\n"); break;
        default: printf("Bad number of samples!\n");
    }
}

void writeRegister8(I2C_CONFIG *i2c, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg; 
    buf[1] = value;
    i2c->write(i2c, HMC5883L_ADDRESS, buf, 2);
}

uint8_t readRegister8(I2C_CONFIG *i2c, uint8_t reg)
{
    uint8_t value;
    if (i2c->read(i2c, HMC5883L_ADDRESS, reg, &value, 1) != ESP_OK)
        return ESP_FAIL;
    return value;
}

int16_t readRegister16(I2C_CONFIG *i2c, uint8_t reg)
{
    int16_t value;
    uint8_t buf[2];
    if (i2c->read(i2c, HMC5883L_ADDRESS, reg, buf, 2) != ESP_OK)
        return ESP_FAIL;

    value = buf[0] << 8 | buf[1];

    return value;
}

