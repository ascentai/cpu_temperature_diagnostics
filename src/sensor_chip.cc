#include "cpu_temperature_diagnostics/sensor_chip.hh"

#include <string>

#include <sensors/sensors.h>

#include "cpu_temperature_diagnostics/temperature_feature.hh"

namespace cpu_temperature_diagnostics
{
SensorChip::SensorChip(const sensors_chip_name* chip,
                       double default_critical_temp,
                       double defaut_max_temp)
  : chip_name_(chip)
{
    int feature_number = 0;
    const sensors_feature* feature;
    while ((feature = sensors_get_features(chip_name_, &feature_number)) != nullptr)
    {
        auto feat = TemperatureFeature::make_temp_feature(
            chip_name_, feature, default_critical_temp, defaut_max_temp);
        if (feat.has_value())
        {
            temperature_features_.push_back(*feat);
        }
    }
}

std::vector<temperature_info> SensorChip::get_temperature_readings() const
{
    std::vector<temperature_info> readings;
    readings.reserve(temperature_features_.size());
    for (const auto& temperature_feature : temperature_features_)
    {
        auto temp_info = temperature_feature.read_temperature_info(chip_name_);
        readings.push_back(temp_info);
    }
    return readings;
}

std::string SensorChip::get_identifier() const
{
    return std::string(chip_name_->prefix) + "-" +
           std::to_string(chip_name_->addr);
}

void SensorChip::critical_temp_override(int critical_temp)
{
    for (auto& feature : temperature_features_)
    {
        feature.set_critical_temp(critical_temp);
    }
}
void SensorChip::max_temp_override(int max_temp)
{
    for (auto& feature : temperature_features_)
    {
        feature.set_max_temp(max_temp);
    }
}
}  // namespace cpu_temperature_diagnostics
