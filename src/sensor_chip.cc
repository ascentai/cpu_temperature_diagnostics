// Copyright (c) 2021, Ascent Robotics, Inc.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Ascent Robotics, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author Thomas Kostas/thomas.kostas@ascent.ai

#include "cpu_temperature_diagnostics/sensor_chip.hh"

#include <string>

#include <sensors/sensors.h>

#include "cpu_temperature_diagnostics/temperature_feature.hh"

namespace cpu_temperature_diagnostics
{
SensorChip::SensorChip(const sensors_chip_name* chip,
                       double default_critical_temp,
                       double default_max_temp)
  : chip_name_(chip)
{
    int feature_number = 0;
    for (const sensors_feature* feature =
             sensors_get_features(chip_name_, &feature_number);
         feature != nullptr;
         feature = sensors_get_features(chip_name_, &feature_number))
    {
        if (auto feat = TemperatureFeature::make_temp_feature(
                chip_name_, feature, default_critical_temp, default_max_temp))
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

void SensorChip::critical_temp_override(double critical_temp)
{
    for (auto& feature : temperature_features_)
    {
        feature.set_critical_temp(critical_temp);
    }
}
void SensorChip::max_temp_override(double max_temp)
{
    for (auto& feature : temperature_features_)
    {
        feature.set_max_temp(max_temp);
    }
}
namespace sensors_chip_factory
{
std::vector<SensorChip> get_chips_with_prefix(const std::string& name)
{
    std::vector<SensorChip> valid_chips;
    int chip_number = 0;
    for (const sensors_chip_name* chip_name =
             sensors_get_detected_chips(nullptr, &chip_number);
         chip_name != nullptr;
         chip_name = sensors_get_detected_chips(nullptr, &chip_number))
    {
        if (std::string(chip_name->prefix) == name)
        {
            valid_chips.emplace_back(chip_name);
        }
    }
    return valid_chips;
}
}  // namespace sensors_chip_factory
}  // namespace cpu_temperature_diagnostics
