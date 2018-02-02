/* Madgwick algorithm: https://github.com/kriswiner/MPU9150/blob/master/MPU9150BasicAHRS.ino
 * https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L.h
 */
#include <stdio.h>
#include <math.h>
#include "sensor.h"
#include "bus.h"
#include "hmc5883l.h"
#include "sdkconfig.h"
#include "ssd1306.h"


#define OLED_SDI  12
#define OLED_SDO  13
#define OLED_SCK  14
#define OLED_CS   15
#define OLED_DC   27
#define OLED_RES  25
#define COMPASSADDR   HMC5883L_ADDRESS
#define IMUADDR   MPU6050_ADDRESS_AD0_LOW

HMC5883L compass;
SSD1306 ssd1306;
Bus bus;

#if 0
void Task_IMU(void *arg)
{
    Vector norm;
    uint8_t cnt = 0;
    float heading;
    float declinationAngle = (3.0 + (48.0 / 60.0)) / (180 / M_PI);

    while (true)
    {
        i2c.setAddr(&i2c, IMUADDR);
        mpu.getMotion(&mpu);
        if (cnt == 30)
        {
            cnt = 0;
            i2c.setAddr(&i2c, COMPASSADDR);
            norm = compass.readNormalize(&compass);
            // You can find your declination on: http://magnetic-declination.com/
            heading = atan2(norm.y, norm.x);
            heading -= declinationAngle;
            if (heading < 0)
                heading += 2 * M_PI;
 
            if (heading > 2 * M_PI)
                heading -= 2 * M_PI;
  
            //printf("m(%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)\n",
              //  raw.x, raw.y, raw.z, norm.x, norm.y, norm.z); 
            printf("head: %f\n", heading * 180/M_PI); 
        }
        //printf("a(%d, %d, %d), g(%d, %d, %d)\n", 
          //      mpu.ax, mpu.ay, mpu.az,
            //    mpu.gx, mpu.gy, mpu.gz);
        cnt++;
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}
#endif

void init()
{
    Bus_Init(&bus);
    Sensor_Init(&bus);

//    i2c.setAddr(&i2c, COMPASSADDR);
//    compass.i2c = &i2c;
//    ret = HMC5883L_Init(&compass);
//    if (ret != ESP_OK)
//        printf("Compass fail\n");

//    compass.checkSettings(&compass);

//    ssd1306.sdi = OLED_SDI;
//    ssd1306.sdo = OLED_SDO;
//    ssd1306.sck = OLED_SCK;
//    ssd1306.cs  = OLED_CS;
//    ssd1306.dc  = OLED_DC;
//    ssd1306.res = OLED_RES;
//    SSD1306_init(&ssd1306);
//    ssd1306.SSD1306_begin(&ssd1306);
//    ssd1306.SSD1306_bitmap(&ssd1306, 0, 0, waveshare, 128, 64);
//    ssd1306.SSD1306_display(&ssd1306);
}

void app_main()
{
    init();

    //xTaskCreate(Task_IMU, "Task-IMU", 2048, NULL, 2, NULL);
}

