#include "Bat_MAX17043.h"

esp_err_t _begin(MAX17043 *self);
esp_err_t _reset(MAX17043 *self);
esp_err_t _quickStart(MAX17043 *self);
uint16_t _getVersion(MAX17043 *self);
float _getBatteryVoltage(MAX17043 *self);
float _getBatteryPercentage(MAX17043 *self);
void _setAlertThreshold(MAX17043 *self, uint8_t percent);
uint8_t _getAlertThreshold(MAX17043 *self);
uint8_t _getAlertThresholdRegister(MAX17043 *self);
bool _isAlerting(MAX17043 *self);
void _clearAlert(MAX17043 *self);
bool _isSleepingRegister(MAX17043 *self);
void _sleep(MAX17043 *self);
void _wake(MAX17043 *self);

void MAX17043_Init(MAX17043 *fuel_gauge)
{
	fuel_gauge->i2c.ADDR = MAX17043_ADDRESS;
	fuel_gauge->i2c.MODE = I2C_MODE_MASTER;
	fuel_gauge->i2c.SDA = fuel_gauge->SDAPin;
	fuel_gauge->i2c.SCL = fuel_gauge->SCLPin;
	fuel_gauge->i2c.SPEED = 400000;
	i2c_init(&fuel_gauge->i2c);

	gpio_config_t conf_gpio;
	conf_gpio.intr_type = GPIO_PIN_INTR_POSEDGE;
	conf_gpio.pin_bit_mask = (1 << fuel_gauge->alertPin);
	conf_gpio.mode = GPIO_MODE_INPUT;
	conf_gpio.pull_up_en = 1;
	gpio_config(&conf_gpio);
	                
	fuel_gauge->begin = _begin;
	fuel_gauge->reset = _reset;
	fuel_gauge->quickStart = _quickStart;
	fuel_gauge->getVersion = _getVersion;
	fuel_gauge->getBatteryVoltage = _getBatteryVoltage;
	fuel_gauge->getBatteryPercentage = _getBatteryPercentage;
	fuel_gauge->setAlertThreshold = _setAlertThreshold;
	fuel_gauge->getAlertThreshold = _getAlertThreshold;
	fuel_gauge->getAlertThresholdRegister = _getAlertThresholdRegister;
	fuel_gauge->isAlerting = _isAlerting;
	fuel_gauge->clearAlert = _clearAlert;
	fuel_gauge->sleep = _sleep;
	fuel_gauge->wake = _wake;
	fuel_gauge->isSleepingRegister = _isSleepingRegister;
}

esp_err_t _begin(MAX17043 *self)
{
	self->setAlertThreshold(self, self->alertThreshold);
	return self->quickStart(self);
}

esp_err_t _reset(MAX17043 *self)
{
	esp_err_t ret;
	uint8_t data[3];
	data[0] = MAX17043_COMMAND;
	data[1] = ((MAX17043_POWER_ON_RESET >> 8) & 0x00FF);
	data[2] = (MAX17043_POWER_ON_RESET & 0x00FF);
	ret = self->i2c.write(&self->i2c, data, 3);
	self->wake(self);

	return ret;
}

esp_err_t _quickStart(MAX17043 *self)
{
	esp_err_t ret;
	uint8_t data[3];
	data[0] = MAX17043_MODE;
	data[1] = ((MAX17043_QUICK_START >> 8) & 0x00FF);
	data[2] = MAX17043_QUICK_START & 0x00FF;
	ret = self->i2c.write(&self->i2c, data, 3);

	return ret;
}

uint16_t _getVersion(MAX17043 *self)
{
	uint16_t value = 0;
	uint8_t data[2] = {0, 0};

	self->i2c.read(&self->i2c, MAX17043_VERSION, data, 2);

	value  = 0xFF00 & (data[0]<<8);
	value |=   0xFF & data[1];

	return value;
}

float _getBatteryVoltage(MAX17043 *self)
{
	uint16_t vcell;
	uint8_t data[2];
	self->i2c.read(&self->i2c, MAX17043_VCELL, data, 2);
	vcell  = 0xFF00 & (data[0]<<8);
	vcell |= 0xFF & data[1];
	vcell = (data[0] << 4) + (data[1] >> 4);

	return (float) vcell * MAX17043_VCELL_VOLTAGE_FACTOR;
}

float _getBatteryPercentage(MAX17043 *self)
{
	uint8_t data[2];
	float percent = 0;
	
	self->i2c.read(&self->i2c, MAX17043_SOC, data, 2);
	percent = data[0];
	percent += ((float)(data[1]))/256;

	return percent;
}

void _setAlertThreshold(MAX17043 *self, uint8_t percent)
{
	uint8_t percentBits = percent;

	if ( percent > (uint8_t) MAX17043_MAX_ALERT_PERCENTAGE )
		percentBits = (uint8_t) MAX17043_MAX_ALERT_PERCENTAGE;
	else if( percent < (uint8_t) MAX17043_MIN_ALERT_PERCENTAGE )
		percentBits = (uint8_t) MAX17043_MIN_ALERT_PERCENTAGE;

	self->alertThreshold = (uint8_t) (MAX17043_MAX_ALERT_PERCENTAGE - percentBits);

	self->wake(self);
}

uint8_t _getAlertThreshold(MAX17043 *self){
  return (uint8_t) (MAX17043_MAX_ALERT_PERCENTAGE - self->alertThreshold);
}

uint8_t _getAlertThresholdRegister(MAX17043 *self)
{
	uint8_t data[2];
	self->i2c.read(&self->i2c, MAX17043_CONFIG, data, 2);
	uint8_t percent = data[1] & 0x1f;

	return (uint8_t) (MAX17043_MAX_ALERT_PERCENTAGE - percent);
}

bool _isAlerting(MAX17043 *self)
{
  return (gpio_get_level(self->alertPin) == 0);
}

void _clearAlert(MAX17043 *self)
{
	if(self->isAlerting(self))
		self->wake(self);
}

void _sleep(MAX17043 *self)
{
	uint8_t data[3];
	data[0] = MAX17043_CONFIG;
	data[1] = ((MAX17043_SLEEP_DEFAULT | self->alertThreshold) >> 8) & 0x00FF;
	data[2] = (MAX17043_SLEEP_DEFAULT | self->alertThreshold) & 0x00FF;
	self->i2c.write(&self->i2c, data, 3);
	self->sleeping = true;
}

void _wake(MAX17043 *self)
{
	uint8_t data[3];
	data[0] = MAX17043_CONFIG;
	data[1] = ((MAX17043_AWAKE_DEFAULT | self->alertThreshold) >> 8) & 0x00FF;
	data[2] = (MAX17043_AWAKE_DEFAULT | self->alertThreshold) & 0x00FF;
	self->i2c.write(&self->i2c, data, 3);
	self->sleeping = false;
}

bool _isSleepingRegister(MAX17043 *self)
{
	uint8_t data[2];

	self->i2c.read(&self->i2c, MAX17043_CONFIG, data, 2);
	uint8_t sleeping = (data[0] >>7) & 0x01;

	return (sleeping == MAX17043_IS_TRUE);
}
