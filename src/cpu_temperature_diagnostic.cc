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

#include "cpu_temperature_diagnostics/cpu_temperature_diagnostic.hh"

#include <string>

#include "cpu_temperature_diagnostics/sensor_chip.hh"

namespace cpu_temperature_diagnostics
{
CpuTemperatureDiagnostic::CpuTemperatureDiagnostic(SensorChip& chip)
  : chip_(chip)
{
    diagnostics_updater_.setHardwareID(chip_.get_identifier());

    auto diagnostic_pub_func =
        std::bind(&CpuTemperatureDiagnostic::diagnostics_callback,
                  this,
                  std::placeholders::_1);

    diagnostics_updater_.add("CPU temperature", diagnostic_pub_func);
}

void CpuTemperatureDiagnostic::diagnostics_callback(
    diagnostic_updater::DiagnosticStatusWrapper& stat) const
{
    auto is_ok = true;
    auto readings = chip_.get_temperature_readings();
    for (const auto& read : readings)
    {
        if (!read.is_ok_)
        {
            is_ok = read.is_ok_;
        }
        std::string status_string = read.is_ok_ ? "OK" : "Error";
        stat.add(read.label_,
                 std::to_string(static_cast<int>(read.temperature_)) + "C " +
                     status_string);
    }
    std::string summary =
        is_ok ? "CPU Temperature is OK" : "CPU Temperature is too hot";

    is_ok ? stat.summary(diagnostic_msgs::DiagnosticStatus::OK, summary) :
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, summary);
}

void CpuTemperatureDiagnostic::emit_diagnostics()
{
    diagnostics_updater_.update();
}

void CpuTemperatureDiagnostic::critical_temp_override(int critical_temp)
{
    chip_.critical_temp_override(critical_temp);
}

void CpuTemperatureDiagnostic::max_temp_override(int max_temp)
{
    chip_.max_temp_override(max_temp);
}

}  // namespace cpu_temperature_diagnostics
