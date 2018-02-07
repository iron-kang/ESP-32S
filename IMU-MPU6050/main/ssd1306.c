#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "ssd1306.h"

#define vccstate SSD1306_SWITCHCAPVCC
#define width 128
#define height 64
#define pages 8

uint8_t pin_dc;

void _SSD1306_begin(struct _SSD1306 *self);
void _SSD1306_display(struct _SSD1306 *self);
void _SSD1306_clear(struct _SSD1306 *self);
void _SSD1306_pixel(struct _SSD1306 *self, int x,int y,char color);
void _SSD1306_bitmap(struct _SSD1306 *self, 
                     unsigned char x,unsigned char y,const unsigned char *pBmp,
                     unsigned char chWidth,unsigned char chHeight);
void _SSD1306_string(struct _SSD1306 *self,
                     unsigned char x,unsigned  char y, const char *pString,
                     unsigned char Size,unsigned char Mode);
void _SSD1306_char(struct _SSD1306 *self, 
                         unsigned char x,unsigned char y,char acsii,char size,char mode);
void _SSD1306_char1616(struct _SSD1306 *self,
                       unsigned char x,unsigned char y,unsigned char chChar);
void _SSD1306_char3216(struct _SSD1306 *self,
                       unsigned char x,unsigned char y,unsigned char chChar);
void _command(struct _SSD1306 *self, uint8_t cmd);
void spi_pre_transfer_callback(spi_transaction_t *t);

void SSD1306_init(SSD1306 *ssd1306)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = ssd1306->sdo,
        .mosi_io_num = ssd1306->sdi,
        .sclk_io_num = ssd1306->sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000,             
        .mode=0,                              
        .spics_io_num = ssd1306->cs,               
        .queue_size = 7,                         
        .pre_cb = spi_pre_transfer_callback,
    };


    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &ssd1306->spi);
    assert(ret == ESP_OK);

    gpio_set_direction(ssd1306->dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(ssd1306->res, GPIO_MODE_OUTPUT);
    pin_dc = ssd1306->dc;

    ssd1306->command = _command;
    ssd1306->SSD1306_begin = _SSD1306_begin;
    ssd1306->SSD1306_display = _SSD1306_display;
    ssd1306->SSD1306_clear = _SSD1306_clear;
    ssd1306->SSD1306_pixel = _SSD1306_pixel;
    ssd1306->SSD1306_bitmap = _SSD1306_bitmap;
    ssd1306->SSD1306_string = _SSD1306_string;
    ssd1306->SSD1306_char1616 = _SSD1306_char1616;
    ssd1306->SSD1306_char3216 = _SSD1306_char3216;
    ssd1306->SSD1306_char = _SSD1306_char;

}

void _command(struct _SSD1306 *self, uint8_t cmd)
{
    gpio_set_level(self->dc, 0);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       
    t.length = 8;               
    t.tx_buffer = &cmd;              
    t.user = (void*) 0;           
    ret = spi_device_transmit(self->spi, &t);  
    assert(ret == ESP_OK);  
}

void spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc = (int)t->user;
    gpio_set_level(pin_dc, dc);
}

void _SSD1306_begin(struct _SSD1306 *self)
{
    //Reset the display
    gpio_set_level(self->res, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(self->res, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    self->command(self, SSD1306_DISPLAYOFF);
    self->command(self, SSD1306_SETDISPLAYCLOCKDIV);
    self->command(self, 0x80);
    self->command(self, SSD1306_SETMULTIPLEX);
    self->command(self, 0x3F);
    self->command(self, SSD1306_SETDISPLAYOFFSET);
    self->command(self, 0x0);                               // no offset
    self->command(self, SSD1306_SETSTARTLINE | 0x0);        // line #0
    self->command(self, SSD1306_CHARGEPUMP);
    self->command(self, (vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

    self->command(self, SSD1306_MEMORYMODE);
    self->command(self, 0x00);                              // 0x0 act like ks0108

    self->command(self, SSD1306_SEGREMAP | 0x1);
    self->command(self, SSD1306_COMSCANDEC);
    self->command(self, SSD1306_SETCOMPINS);
    self->command(self, 0x12);           // TODO - calculate based on _rawHieght ?
    self->command(self, SSD1306_SETCONTRAST);
    self->command(self, (vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
    self->command(self, SSD1306_SETPRECHARGE);
    self->command(self, (vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
    self->command(self, SSD1306_SETVCOMDETECT);
    self->command(self, 0x40);
    self->command(self, SSD1306_DISPLAYALLON_RESUME);
    self->command(self, SSD1306_NORMALDISPLAY);
    self->command(self, SSD1306_DISPLAYON);
}

void _SSD1306_display(struct _SSD1306 *self)
{
    self->command(self, SSD1306_COLUMNADDR);
    self->command(self, 0);         //cloumn start address
    self->command(self, width -1); //cloumn end address
    self->command(self, SSD1306_PAGEADDR);
    self->command(self, 0);         //page atart address
    self->command(self, pages -1); //page end address

    gpio_set_level(self->dc, 1);
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = sizeof(self->buffer)*8;      //Len is in bytes, transaction length is in bits.
    t.tx_buffer = self->buffer;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret = spi_device_transmit(self->spi, &t);  //Transmit!
    assert(ret==ESP_OK);  
}

void _SSD1306_clear(struct _SSD1306 *self)
{
    int i;
    for(i = 0;i<sizeof(self->buffer);i++)
    {
        self->buffer[i] = 0;
    }
}

void _SSD1306_pixel(struct _SSD1306 *self, int x,int y,char color)
{
    if(x > width || y > height)return ;
    if(color)
        self->buffer[x+(y/8)*width] |= 1<<(y%8);
    else
        self->buffer[x+(y/8)*width] &= ~(1<<(y%8));
}

void _SSD1306_bitmap(struct _SSD1306 *self, 
                     unsigned char x,unsigned char y,const unsigned char *pBmp,
                     unsigned char chWidth,unsigned char chHeight)
{
    unsigned char i,j,byteWidth = (chWidth + 7)/8;
    for(j = 0;j < chHeight;j++){
        for(i = 0;i <chWidth;i ++){
            if(*(pBmp +j*byteWidth+i/8) & (128 >> (i & 7))){
                self->SSD1306_pixel(self, x+i,y+j,1);
            }
        }
    }
}

void _SSD1306_string(struct _SSD1306 *self,
                     unsigned char x,unsigned  char y, const char *pString,
                     unsigned char Size,unsigned char Mode)
{
    while (*pString != '\0') {       
        if (x > (width - Size / 2)) {
            x = 0;
            y += Size;
            if (y > (height - Size)) {
                y = x = 0;
            }
        }
		
        self->SSD1306_char(self, x, y, *pString, Size, Mode);
        x += Size / 2;
        pString ++;
    }
}

void _SSD1306_char1616(struct _SSD1306 *self,
                       unsigned char x,unsigned char y,unsigned char chChar)
{
    unsigned char i, j;
    unsigned char chTemp = 0, y0 = y, chMode = 0;

    for (i = 0; i < 32; i ++) {
        chTemp = Font1612[chChar - 0x30][i];
        for (j = 0; j < 8; j ++) {
            chMode = chTemp & 0x80? 1 : 0; 
            self->SSD1306_pixel(self, x, y, chMode);
            chTemp <<= 1;
            y ++;
            if ((y - y0) == 16) {
                y = y0;
                x++;
                break;
            }
        }
    }
}
void _SSD1306_char3216(struct _SSD1306 *self,
                       unsigned char x,unsigned char y,unsigned char chChar)
{
    unsigned char i, j;
    unsigned char chTemp = 0, y0 = y, chMode = 0;

    for (i = 0; i < 64; i ++) {
        chTemp = Font3216[chChar - 0x30][i];
        for (j = 0; j < 8; j ++) {
            chMode = chTemp & 0x80? 1 : 0; 
            self->SSD1306_pixel(self, x, y, chMode);
            chTemp <<= 1;
            y++;
            if ((y - y0) == 32) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

void _SSD1306_char(struct _SSD1306 *self, 
                   unsigned char x,unsigned char y,char acsii,char size,char mode)
{
    unsigned char i,j,y0=y;
    char temp;
    unsigned char ch = acsii - ' ';
    for(i = 0;i<size;i++)
    {
        if(size == 12)
        {
            if(mode)temp=Font1206[ch][i];
            else temp = ~Font1206[ch][i];
        }
        else 
        {			
            if(mode)temp=Font1608[ch][i];
            else temp = ~Font1608[ch][i];
        }
        for(j =0;j<8;j++)
        {
            if(temp & 0x80) self->SSD1306_pixel(self, x, y, 1);
            else self->SSD1306_pixel(self, x, y, 0);
            temp <<=1;
            y++;
            if((y-y0)==size)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

