#include "barometer.h"
#include "system.h"
#include "logger.h"

CBarometer::CBarometer()
{
    m_matter_update_by_client_clus_pressuremeasure_attr_measureval = false;
}

bool CBarometer::matter_init_endpoint()
{
    esp_matter::node_t *root = GetSystem()->get_root_node();
    esp_matter::endpoint::pressure_sensor::config_t config_endpoint;
    uint8_t flags = esp_matter::ENDPOINT_FLAG_DESTROYABLE;
    m_endpoint = esp_matter::endpoint::pressure_sensor::create(root, &config_endpoint, flags, nullptr);
    if (!m_endpoint) {
        GetLogger(eLogType::Error)->Log("Failed to create endpoint");
        return false;
    }
    return CDevice::matter_init_endpoint();
}

bool CBarometer::matter_config_attributes()
{
    return true;
}

bool CBarometer::set_pressure_measurement_min_measured_value(int16_t value)
{
    esp_matter::cluster_t *cluster = esp_matter::cluster::get(m_endpoint, chip::app::Clusters::PressureMeasurement::Id);
    if (!cluster) {
        GetLogger(eLogType::Error)->Log("Failed to get PressureMeasurement cluster");
        return false;
    }
    esp_matter::attribute_t *attribute = esp_matter::attribute::get(cluster, chip::app::Clusters::PressureMeasurement::Attributes::MinMeasuredValue::Id);
    if (!attribute) {
        GetLogger(eLogType::Error)->Log("Failed to get MinMeasuredValue attribute");
        return false;
    }
    esp_matter_attr_val_t val = esp_matter_nullable_int16(value);
    esp_err_t ret = esp_matter::attribute::set_val(attribute, &val);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Warning)->Log("Failed to set MaxMeasuredValue attribute value (ret: %d)", ret);
    }

    return true;
}

bool CBarometer::set_pressure_measurement_max_measured_value(int16_t value)
{
    esp_matter::cluster_t *cluster = esp_matter::cluster::get(m_endpoint, chip::app::Clusters::PressureMeasurement::Id);
    if (!cluster) {
        GetLogger(eLogType::Error)->Log("Failed to get PressureMeasurement cluster");
        return false;
    }
    esp_matter::attribute_t *attribute = esp_matter::attribute::get(cluster, chip::app::Clusters::PressureMeasurement::Attributes::MaxMeasuredValue::Id);
    if (!attribute) {
        GetLogger(eLogType::Error)->Log("Failed to get MaxMeasuredValue attribute");
        return false;
    }
    esp_matter_attr_val_t val = esp_matter_nullable_int16(value);
    esp_err_t ret = esp_matter::attribute::set_val(attribute, &val);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Warning)->Log("Failed to set MaxMeasuredValue attribute value (ret: %d)", ret);
    }

    return true;
}

void CBarometer::matter_on_change_attribute_value(esp_matter::attribute::callback_type_t type, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *value)
{
    if (cluster_id == chip::app::Clusters::PressureMeasurement::Id) {
        if (attribute_id == chip::app::Clusters::PressureMeasurement::Attributes::MeasuredValue::Id) {
            if (m_matter_update_by_client_clus_pressuremeasure_attr_measureval) {
                m_matter_update_by_client_clus_pressuremeasure_attr_measureval = false;
            }
        }
    }
}

void CBarometer::matter_update_all_attribute_values()
{
    matter_update_clus_pressuremeasure_attr_measureval();
}

void CBarometer::update_measured_value_pressure(float value)
{
    m_measured_value_pressure = (int16_t)(value / 100.f);   // Pa -> hPa
    if (m_measured_value_pressure != m_measured_value_pressure_prev) {
        GetLogger(eLogType::Info)->Log("Update measured pressure value as %g", value);
        matter_update_clus_pressuremeasure_attr_measureval();
    }
    m_measured_value_pressure_prev = m_measured_value_pressure;
}

void CBarometer::matter_update_clus_pressuremeasure_attr_measureval(bool force_update/*=false*/)
{
    esp_matter_attr_val_t target_value = esp_matter_nullable_int16(m_measured_value_pressure);
    matter_update_cluster_attribute_common(
        m_endpoint_id,
        chip::app::Clusters::PressureMeasurement::Id,
        chip::app::Clusters::PressureMeasurement::Attributes::MeasuredValue::Id,
        target_value,
        &m_matter_update_by_client_clus_pressuremeasure_attr_measureval,
        force_update
    );
}
