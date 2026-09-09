#ifndef BMP390_H
#define BMP390_H
#include <stdint.h>
#include "driver/i2c.h"

typedef struct {
    float temperature;
    float pressure;
    float altitude;
    bool new_reading; 
    uint32_t timestamp;  
} BMP390;

typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
} BMP390_Calib;

void init_bmp390(BMP390 *bmp390, BMP390_Calib *calib); // sets bmp390 reading to zero and wakes it up
void wake_up_bmp390();
void write_bmp390(uint8_t reg, uint8_t data);
uint8_t read_bmp390(uint8_t reg);
void bmp390_read_all(BMP390 *bmp390, BMP390_Calib *calib);
void bmp390_read_calib(BMP390_Calib *calib);
esp_err_t bmp390_read_bytes(uint8_t reg_addr, uint8_t *data, size_t length);

extern BMP390_Calib calib;
extern BMP390 bmp390;
#endif // BMP390_H