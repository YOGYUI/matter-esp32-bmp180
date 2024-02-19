#pragma once
#ifndef _BAROMETER_H_
#define _BAROMETER_H_

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

class CBarometer : public CDevice
{
public:
    CBarometer();

    bool matter_init_endpoint() override;
    bool matter_config_attributes() override;
    void matter_on_change_attribute_value(
        esp_matter::attribute::callback_type_t type,
        uint32_t cluster_id,
        uint32_t attribute_id,
        esp_matter_attr_val_t *value
    ) override;
    void matter_update_all_attribute_values() override;

public:
    bool set_pressure_measurement_min_measured_value(int16_t value);
    bool set_pressure_measurement_max_measured_value(int16_t value);

    void update_measured_value_pressure(float value) override;

private:
    bool m_matter_update_by_client_clus_pressuremeasure_attr_measureval;

    void matter_update_clus_pressuremeasure_attr_measureval(bool force_update = false);
};

#ifdef __cplusplus
};
#endif
#endif