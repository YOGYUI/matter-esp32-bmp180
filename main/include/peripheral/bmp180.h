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
    ULTRA_HIGH_RESOLUTION = 3,
} eBMP180Mode;

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
    bool read_measurement(float *pressure);

private:
    static CBmp180Ctrl *_instance;
    CI2CMaster *m_i2c_master;

    int16_t m_caldat_ac1, m_caldat_ac2, m_caldat_ac3;
    uint16_t m_caldat_ac4, m_caldat_ac5, m_caldat_ac6;
    int16_t m_caldat_b1, m_caldat_b2;
    int16_t m_caldat_mb, m_caldat_mc, m_caldat_md;

    bool read_chip_id(uint8_t *chip_id);
    bool read_calibration_data();

    int32_t calculate_value_b5(int32_t value_ut);
    bool read_raw_temperature(uint16_t *temperature);
    bool read_raw_pressure(eBMP180Mode mode, uint32_t *pressure);
    bool read_pressure(eBMP180Mode mode, int32_t *pressure);
};

inline CBmp180Ctrl* GetBmp180Ctrl() {
    return CBmp180Ctrl::Instance();
}

#ifdef __cplusplus
}
#endif
#endif