#pragma once
#ifndef _BMP180_H_
#define _BMP180_H_

#include "I2CMaster.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ULTRA_LOW_POWER = 0,
    STANDARD = 1,
    HIGH_RESOLUTION = 2,
    ULTRA_HIGH_RESOLUTION = 3
} eBMP180Mode;

typedef struct bmp180_cal_data {
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    bmp180_cal_data() {
        ac1 = ac2 = ac3 = ac4 = ac5 = ac6 = b1 = b2 = mb = mc = md = 0;
    };
} bmp180_cal_data_t;

class CBmp180Ctrl
{
public:
    CBmp180Ctrl();
    virtual ~CBmp180Ctrl();
    static CBmp180Ctrl* Instance();

public:
    bool initialize(CI2CMaster *i2c_master);
    bool release();

    bool soft_reset();
    bool read_measurement(float *pressure, eBMP180Mode mode = eBMP180Mode::ULTRA_HIGH_RESOLUTION);

    float calculate_true_temperature(int32_t raw_value);
    double calculate_absolute_altutide(int32_t pressure);

private:
    static CBmp180Ctrl *_instance;
    CI2CMaster *m_i2c_master;
    bmp180_cal_data_t m_cal_data;

    bool read_chip_id(uint8_t *chip_id);
    bool read_calibration_data();

    bool read_uncompensated_temperature(int32_t *temperature);
    bool read_uncompensated_pressure(eBMP180Mode mode, int32_t *pressure);
    
    int32_t calculate_value_b5(int32_t value_ut);
    int32_t calculate_true_pressure(int32_t raw_temperature, int32_t raw_pressure, eBMP180Mode mode);
};

inline CBmp180Ctrl* GetBmp180Ctrl() {
    return CBmp180Ctrl::Instance();
}

#ifdef __cplusplus
}
#endif
#endif