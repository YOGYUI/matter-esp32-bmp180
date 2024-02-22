#include "bmp180.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <math.h>

#define BMP180_I2C_ADDR         0x77    /**< BMP180 I2C address */

#define BMP180_CAL_AC1          0xAA    /**< R Calibration data (16 bits) */
#define BMP180_CAL_AC2          0xAC    /**< R Calibration data (16 bits) */
#define BMP180_CAL_AC3          0xAE    /**< R Calibration data (16 bits) */
#define BMP180_CAL_AC4          0xB0    /**< R Calibration data (16 bits) */
#define BMP180_CAL_AC5          0xB2    /**< R Calibration data (16 bits) */
#define BMP180_CAL_AC6          0xB4    /**< R Calibration data (16 bits) */
#define BMP180_CAL_B1           0xB6    /**< R Calibration data (16 bits) */
#define BMP180_CAL_B2           0xB8    /**< R Calibration data (16 bits) */
#define BMP180_CAL_MB           0xBA    /**< R Calibration data (16 bits) */
#define BMP180_CAL_MC           0xBC    /**< R Calibration data (16 bits) */
#define BMP180_CAL_MD           0xBE    /**< R Calibration data (16 bits) */

#define BMP180_CONTROL          0xF4    /**< Control register */
#define BMP180_TEMPDATA         0xF6    /**< Temperature data register */
#define BMP180_PRESSUREDATA     0xF6    /**< Pressure data register */
#define BMP180_READTEMPCMD      0x2E    /**< Read temperature control register value */
#define BMP180_READPRESSURECMD  0x34    /**< Read pressure control register value */
#define BMP180_CHIP_ID          0xD0    /**< Read Chip ID */
#define BMP180_SOFTRSTCMD       0xE0    /**< perform power on reset */

CBmp180Ctrl* CBmp180Ctrl::_instance = nullptr;

CBmp180Ctrl::CBmp180Ctrl()
{
    m_i2c_master = nullptr;
    m_cal_data = bmp180_cal_data_t();
}

CBmp180Ctrl::~CBmp180Ctrl()
{
}

CBmp180Ctrl* CBmp180Ctrl::Instance()
{
    if (!_instance) {
        _instance = new CBmp180Ctrl();
    }

    return _instance;
}

bool CBmp180Ctrl::initialize(CI2CMaster *i2c_master)
{
    m_i2c_master = i2c_master;

    uint8_t chip_id = 0;
    if (!read_chip_id(&chip_id)) {
        GetLogger(eLogType::Error)->Log("Failed to read chip ID");
        return false;
    }
    if (chip_id != 0x55) {
        GetLogger(eLogType::Error)->Log("chip ID mismatch (0x%02X)", chip_id);
        return false;
    }

    if (!read_calibration_data()) {
        GetLogger(eLogType::Error)->Log("Failed to read calibration data");
        return false;
    }

    GetLogger(eLogType::Info)->Log("Initialized");
    return true;
}

bool CBmp180Ctrl::release()
{
    return true;
}

bool CBmp180Ctrl::soft_reset()
{
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[2] = {BMP180_SOFTRSTCMD, 0xB6};
    if (!m_i2c_master->write_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write)))
        return false;

    return true;
}

bool CBmp180Ctrl::read_measurement(float *pressure, eBMP180Mode mode/*=eBMP180Mode::ULTRA_HIGH_RESOLUTION*/)
{
    int32_t raw_temperature = 0;
    int32_t raw_pressure = 0;
    if (!read_uncompensated_temperature(&raw_temperature))
        return false;
    if (!read_uncompensated_pressure(mode, &raw_pressure))
        return false;

    // unit: Pa = 0.01hPa = 0.001kPa
    int32_t conv_val = calculate_true_pressure(raw_temperature, raw_pressure, mode);
    if (pressure)
        *pressure = (float)conv_val;
    
    return true;
}

bool CBmp180Ctrl::read_chip_id(uint8_t *chip_id)
{
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[1] = {BMP180_CHIP_ID};
    uint8_t data_read[1] = {0, };
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    
    if (chip_id) {
        *chip_id = data_read[0];
    }

    return true;
}

bool CBmp180Ctrl::read_calibration_data()
{
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[1] = {0, };
    uint8_t data_read[2] = {0, };
    data_write[0] = BMP180_CAL_AC1;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac1 = (((int16_t)data_read[0]) << 8) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC1: %d", m_cal_data.ac1);

    data_write[0] = BMP180_CAL_AC2;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac2 = (((int16_t)data_read[0]) << 8) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC2: %d", m_cal_data.ac2);

    data_write[0] = BMP180_CAL_AC3;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac3 = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC3: %d", m_cal_data.ac3);

    data_write[0] = BMP180_CAL_AC4;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac4 = (((uint16_t)data_read[0] << 8)) | ((uint16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC4: %u", m_cal_data.ac4);

    data_write[0] = BMP180_CAL_AC5;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac5 = (((uint16_t)data_read[0] << 8)) | ((uint16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC5: %u", m_cal_data.ac5);

    data_write[0] = BMP180_CAL_AC6;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.ac6 = (((uint16_t)data_read[0] << 8)) | ((uint16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data AC6: %u", m_cal_data.ac6);

    data_write[0] = BMP180_CAL_B1;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.b1 = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data B1: %d", m_cal_data.b1);

    data_write[0] = BMP180_CAL_B2;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.b2 = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data B2: %d", m_cal_data.b2);

    data_write[0] = BMP180_CAL_MB;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.mb = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data MB: %d", m_cal_data.mb);

    data_write[0] = BMP180_CAL_MC;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.mc = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data MC: %d", m_cal_data.mc);

    data_write[0] = BMP180_CAL_MD;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_cal_data.md = (((int16_t)data_read[0] << 8)) | ((int16_t)data_read[1]);
    // GetLogger(eLogType::Info)->Log("Calibration Data MD: %d", m_cal_data.md);

    return true;
}

bool CBmp180Ctrl::read_uncompensated_temperature(int32_t *temperature)
{
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[2] = {BMP180_CONTROL, BMP180_READTEMPCMD};
    
    if (!m_i2c_master->write_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write)))
        return false;
    // vTaskDelay(pdMS_TO_TICKS(5));
    vTaskDelay(pdMS_TO_TICKS(100));

    data_write[0] = BMP180_TEMPDATA;
    uint8_t data_read[2] = {0, };
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, 1, data_read, sizeof(data_read)))
        return false;

    if (temperature) {
        *temperature = (((int32_t)data_read[0]) << 8) | ((int32_t)data_read[1]);
    }

    return true;
}

bool CBmp180Ctrl::read_uncompensated_pressure(eBMP180Mode mode, int32_t *pressure)
{
    int32_t raw_pressure;
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[2] = {BMP180_CONTROL, (uint8_t)(BMP180_READPRESSURECMD + ((int)mode << 6))};
    if (!m_i2c_master->write_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write)))
        return false;
    
    /*
    switch (mode) {
    case ULTRA_LOW_POWER:
        vTaskDelay(pdMS_TO_TICKS(5));
        break;
    case STANDARD:
        vTaskDelay(pdMS_TO_TICKS(8));
        break;
    case HIGH_RESOLUTION:
        vTaskDelay(pdMS_TO_TICKS(14));
        break;
    case ULTRA_HIGH_RESOLUTION:
        vTaskDelay(pdMS_TO_TICKS(26));
        break;
    }
    */
    vTaskDelay(pdMS_TO_TICKS(100));

    data_write[0] = BMP180_PRESSUREDATA;
    uint8_t data_read[3] = {0, };
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, 1, data_read, sizeof(data_read)))
        return false;
    raw_pressure = (((int32_t)data_read[0]) << 16) | (((int32_t)data_read[1]) << 8) | ((int32_t)data_read[0]);
    // GetLogger(eLogType::Info)->Log("buffer: %u, %u, %u", data_read[0], data_read[1], data_read[2]);
    raw_pressure = raw_pressure >> (8 - (int)mode);
    
    if (pressure) {
        *pressure = raw_pressure;
    }
    
    return true;
}

int32_t CBmp180Ctrl::calculate_value_b5(int32_t value_ut)
{
    int32_t X1 = (value_ut - (int32_t)m_cal_data.ac6) * ((int32_t)m_cal_data.ac5) >> 15;
    int32_t X2 = (((int32_t)m_cal_data.mc) << 11) / (X1 + (int32_t)m_cal_data.md);

    return X1 + X2;
}

int32_t CBmp180Ctrl::calculate_true_pressure(int32_t raw_temperature, int32_t raw_pressure, eBMP180Mode mode)
{
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;
    int32_t result;

    UT = raw_temperature;
    UP = raw_pressure;

    // GetLogger(eLogType::Info)->Log("uncompensated temperature: %d", raw_temperature);
    // GetLogger(eLogType::Info)->Log("uncompensated pressure: %d", raw_pressure);

    B5 = calculate_value_b5(UT);
    B6 = B5 - 4000;
    // GetLogger(eLogType::Info)->Log("B5: %d", B5);
    // GetLogger(eLogType::Info)->Log("B6: %d", B6);
    
    X1 = ((int32_t)m_cal_data.b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)m_cal_data.ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)m_cal_data.ac1 * 4 + X3) << (int)mode) + 2) / 4;
    // GetLogger(eLogType::Info)->Log("X1: %d, X2: %d, X3: %d, B3: %d", X1, X2, X3, B3);
    
    X1 = ((int32_t)m_cal_data.ac3 * B6) >> 13;
    X2 = ((int32_t)m_cal_data.b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)m_cal_data.ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> (int)mode);
    // GetLogger(eLogType::Info)->Log("X1: %d, X2: %d, X3: %d, B4: %u, B7: %u", X1, X2, X3, B4, B7);

    p = B7 < 0x80000000UL ? (B7 * 2) / B4 : (B7 / B4) * 2;
    // GetLogger(eLogType::Info)->Log("p: %d", p);
    X1 = (p >> 8) * (p >> 8);
    // GetLogger(eLogType::Info)->Log("X1: %d", X1);
    X1 = (X1 * 3038) >> 16;
    // GetLogger(eLogType::Info)->Log("X1: %d", X1);
    X2 = (-7357 * p) >> 16;
    // GetLogger(eLogType::Info)->Log("X2: %d", X2);
    
    result = p + ((X1 + X2 + 3791) >> 4);
    // GetLogger(eLogType::Info)->Log("p: %d", result);
    return result;
}

float CBmp180Ctrl::calculate_true_temperature(int32_t raw_value)
{
    int32_t B5 = calculate_value_b5(raw_value);
    return (float)((B5 + 8) >> 4) / 10.f;
}

double CBmp180Ctrl::calculate_absolute_altutide(int32_t pressure)
{
    double pressure_hpa = (double)pressure / 100.;
    double sealevel_hpa = 1013.25;
    return 44330. * (1 - pow(pressure_hpa/sealevel_hpa, 1. / 5.255));
}