#pragma once
#include <string>
#include <vector>

#include <sensors/sensors.h>

#include "cpu_temperature_diagnostics/temperature_feature.hh"
namespace cpu_temperature_diagnostics
{
class SensorChip
{
  public:
    SensorChip(const sensors_chip_name* chip,
               double default_critical_temp = 100,
               double defaut_max_temp = 85);
    std::vector<temperature_info> get_temperature_readings() const;
    std::string get_identifier() const;
    void critical_temp_override(double critical_temp);
    void max_temp_override(double max_temp);

  private:
    std::vector<TemperatureFeature> temperature_features_;
    const sensors_chip_name* chip_name_;
};

struct sensors_chip_factory
{
    static std::vector<SensorChip> get_chips_with_prefix(std::string& name)
    {
        std::vector<SensorChip> valid_chips;
        sensors_chip_name const* chip_name;
        int chip_number = 0;
        while ((chip_name = sensors_get_detected_chips(0, &chip_number)) !=
               NULL)
        {
            if (std::string(chip_name->prefix) == name)
            {
                valid_chips.push_back(SensorChip(chip_name));
            }
        }
        return valid_chips;
    }
};
}  // namespace cpu_temperature_diagnostics