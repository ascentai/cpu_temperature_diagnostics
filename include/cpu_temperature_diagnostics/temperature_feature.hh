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
#include <optional>

#include <sensors/sensors.h>

namespace cpu_temperature_diagnostics
{
struct temperature_info
{
    std::string label_;
    double temperature_ = 0;
    bool is_ok_ = false;
};

class TemperatureFeature
{
  public:
    static std::optional<TemperatureFeature>
    make_temp_feature(const sensors_chip_name* chip_name,
                      const sensors_feature* feature,
                      double default_critical_temp = 100,
                      double defaut_max_temp = 85);
    temperature_info
    read_temperature_info(const sensors_chip_name* chip_name) const;
    std::string get_label() const;

    void set_critical_temp(double critical_temp)
    {
        crit_temp_ = critical_temp;
    }

    void set_max_temp(double max_temp)
    {
        max_temp_ = max_temp;
    }

  private:
    TemperatureFeature(const sensors_chip_name* chip_name,
                       const sensors_feature* feature,
                       double default_critical_temp,
                       double defaut_max_temp);
    std::string label_;
    double max_temp_;
    double crit_temp_;
    const sensors_subfeature* input_temp_subfeature_;
};
}  // namespace cpu_temperature_diagnostics