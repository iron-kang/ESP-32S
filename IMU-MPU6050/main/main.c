/* Madgwick algorithm: https://github.com/kriswiner/MPU9150/blob/master/MPU9150BasicAHRS.ino
 * https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L.h
 */
#include <stdio.h>
#include "mpu6050.h"
#include "sdkconfig.h"
#include "ssd1306.h"

#define MPU6050_SDA 18
#define MPU6050_SCL 19
#define OLED_SDI  12
#define OLED_SDO  13
#define OLED_SCK  14
#define OLED_CS   15
#define OLED_DC   27
#define OLED_RES  25

MPU6050 mpu;
SSD1306 ssd1306;

void Task_IMU(void *arg)
{
    while (true)
    {
        mpu.getMotion(&mpu);
        printf("a(%d, %d, %d), g(%d, %d, %d)\n", 
                mpu.ax, mpu.ay, mpu.az,
                mpu.gx, mpu.gy, mpu.gz);
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void init()
{
    esp_err_t ret;

    mpu.SDAPin = MPU6050_SDA;
    mpu.SCLPin = MPU6050_SCL;
    ret = MPU6050_Init(&mpu);

    if (ret != ESP_OK)
        printf("mpu6050 fail\n");

    ssd1306.sdi = OLED_SDI;
    ssd1306.sdo = OLED_SDO;
    ssd1306.sck = OLED_SCK;
    ssd1306.cs  = OLED_CS;
    ssd1306.dc  = OLED_DC;
    ssd1306.res = OLED_RES;
    SSD1306_init(&ssd1306);
    ssd1306.SSD1306_begin(&ssd1306);
    ssd1306.SSD1306_bitmap(&ssd1306, 0, 0, waveshare, 128, 64);
    ssd1306.SSD1306_display(&ssd1306);
}

void app_main()
{
    init();

    xTaskCreate(Task_IMU, "Task-IMU", 2048, NULL, 2, NULL);
}

