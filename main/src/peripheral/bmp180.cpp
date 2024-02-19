#include "bmp180.h"
#include "logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

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

CBmp180Ctrl* CBmp180Ctrl::_instance = nullptr;

CBmp180Ctrl::CBmp180Ctrl()
{
    m_i2c_master = nullptr;
    m_caldat_ac1 = m_caldat_ac2 = m_caldat_ac3 = 0;
    m_caldat_ac4 = m_caldat_ac5 = m_caldat_ac6 = 0;
    m_caldat_b1 = m_caldat_b2 = 0;
    m_caldat_mb = m_caldat_mc = m_caldat_md = 0;
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
    return true;
}

bool CBmp180Ctrl::read_measurement(float *pressure)
{
    int32_t val = 0;  // unit: Pa
    if (!read_pressure(ULTRA_HIGH_RESOLUTION, &val))
        return false;

    if (pressure)
        *pressure = (float)val;
    
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
    m_caldat_ac1 = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC1: %d", m_caldat_ac1);

    data_write[0] = BMP180_CAL_AC2;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_ac2 = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC2: %d", m_caldat_ac2);

    data_write[0] = BMP180_CAL_AC3;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_ac3 = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC3: %d", m_caldat_ac3);

    data_write[0] = BMP180_CAL_AC4;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_ac4 = ((uint16_t)data_read[0] << 8) | (uint16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC4: %u", m_caldat_ac4);

    data_write[0] = BMP180_CAL_AC5;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_ac5 = ((uint16_t)data_read[0] << 8) | (uint16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC5: %u", m_caldat_ac5);

    data_write[0] = BMP180_CAL_AC6;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_ac6 = ((uint16_t)data_read[0] << 8) | (uint16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data AC6: %u", m_caldat_ac6);

    data_write[0] = BMP180_CAL_B1;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_b1 = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data B1: %d", m_caldat_b1);

    data_write[0] = BMP180_CAL_B2;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_b2 = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data B2: %d", m_caldat_b2);

    data_write[0] = BMP180_CAL_MB;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_mb = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data MB: %d", m_caldat_mb);

    data_write[0] = BMP180_CAL_MC;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_mc = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data MC: %d", m_caldat_mc);

    data_write[0] = BMP180_CAL_MD;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write), data_read, sizeof(data_read)))
        return false;
    m_caldat_md = ((int16_t)data_read[0] << 8) | (int16_t)data_read[1];
    GetLogger(eLogType::Info)->Log("Calibration Data MD: %d", m_caldat_md);

    return true;
}

int32_t CBmp180Ctrl::calculate_value_b5(int32_t value_ut)
{
    int32_t X1 = (value_ut - (int32_t)m_caldat_ac6) * ((int32_t)m_caldat_ac5) >> 15;
    int32_t X2 = ((int32_t)m_caldat_mc << 11) / (X1 + (int32_t)m_caldat_md);
    
    return X1 + X2;
}

bool CBmp180Ctrl::read_raw_temperature(uint16_t *temperature)
{
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[2] = {BMP180_CONTROL, BMP180_READTEMPCMD};
    
    if (!m_i2c_master->write_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write)))
        return false;
    vTaskDelay(5 / portTICK_PERIOD_MS);

    data_write[0] = BMP180_TEMPDATA;
    uint8_t data_read[2] = {0, };
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, 1, data_read, sizeof(data_read)))
        return false;

    if (temperature) {
        *temperature = ((uint16_t)data_read[0] << 8) | (uint16_t)data_read[1];
    }

    return true;
}

bool CBmp180Ctrl::read_raw_pressure(eBMP180Mode mode, uint32_t *pressure)
{
    uint32_t raw_pressure;
    if (!m_i2c_master) {
        GetLogger(eLogType::Error)->Log("I2C Controller is null");
        return false;
    }

    uint8_t data_write[2] = {BMP180_CONTROL, (uint8_t)(BMP180_READPRESSURECMD + (mode << 6))};
    if (!m_i2c_master->write_bytes(BMP180_I2C_ADDR, data_write, sizeof(data_write)))
        return false;
    
    switch (mode) {
    case ULTRA_LOW_POWER:
        vTaskDelay(5 / portTICK_PERIOD_MS);
        break;
    case STANDARD:
        vTaskDelay(8 / portTICK_PERIOD_MS);
        break;
    case HIGH_RESOLUTION:
        vTaskDelay(14 / portTICK_PERIOD_MS);
        break;
    case ULTRA_HIGH_RESOLUTION:
        vTaskDelay(26 / portTICK_PERIOD_MS);
        break;
    }

    uint8_t data_read[2] = {0, };
    data_write[0] = BMP180_PRESSUREDATA;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, 1, data_read, sizeof(data_read)))
        return false;
    raw_pressure = ((uint32_t)data_read[0] << 8) | (uint32_t)data_read[1];
    raw_pressure <<= 8;
    
    data_write[0] = BMP180_PRESSUREDATA + 2;
    if (!m_i2c_master->write_and_read_bytes(BMP180_I2C_ADDR, data_write, 1, data_read, 1))
        return false;
    raw_pressure |= (uint32_t)data_read[0];
    raw_pressure >>= (8 - mode);

    if (pressure) {
        *pressure = raw_pressure;
    }
    
    return true;
}

bool CBmp180Ctrl::read_pressure(eBMP180Mode mode, int32_t *pressure)
{
    uint16_t raw_temperature = 0;
    uint32_t raw_pressure = 0;
    if (!read_raw_temperature(&raw_temperature))
        return false;
    if (!read_raw_pressure(mode, &raw_pressure))
        return false;

    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    UT = (int32_t)raw_temperature;
    UP = (int32_t)raw_pressure;
    B5 = calculate_value_b5(UT);
    B6 = B5 - 4000;
    
    X1 = ((int32_t)m_caldat_b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)m_caldat_ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)m_caldat_ac1 * 4 + X3) << mode) + 2) / 4;
    
    X1 = ((int32_t)m_caldat_ac3 * B6) >> 13;
    X2 = ((int32_t)m_caldat_b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)m_caldat_ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> mode);

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + (int32_t)3791) >> 4);

    if (pressure)
        *pressure = p;
    
    return true;
}