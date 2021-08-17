// BSD License
//
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
    /**
     * @brief Construct a new Sensor Chip object
     *
     * @param chip  chip that the SensorChip object is abstracting
     * @param default_critical_temp Critical temp used if not provided by the
     * chip
     * @param default_max_temp Max temp used if not provided by the chip
     */
    SensorChip(const sensors_chip_name* chip,
               double default_critical_temp = 100,
               double default_max_temp = 85);

    std::vector<temperature_info> get_temperature_readings() const;
    std::string get_identifier() const;
    void critical_temp_override(double critical_temp);
    void max_temp_override(double max_temp);

  private:
    std::vector<TemperatureFeature> temperature_features_;
    const sensors_chip_name* chip_name_;
};

namespace sensors_chip_factory
{
std::vector<SensorChip> get_chips_with_prefix(const std::string& name);
}  // namespace sensors_chip_factory

}  // namespace cpu_temperature_diagnostics
