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
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "cpu_temperature_diagnostics/sensor_chip.hh"

namespace cpu_temperature_diagnostics
{
class CpuTemperatureDiagnostic
{
  public:
    explicit CpuTemperatureDiagnostic(SensorChip& chip);
    void emit_diagnostics();
    /**
     * @brief Overrides the chip read critical temperature
     *
     * @param critical_temp New critical temperature
     */
    void critical_temp_override(int critical_temp);

    /**
     * @brief Overrides the chip read maximum temperature
     *
     * @param max_temp New maximum temperature
     */
    void max_temp_override(int max_temp);

  private:
    void diagnostics_callback(
        diagnostic_updater::DiagnosticStatusWrapper& stat) const;

    SensorChip chip_;
    diagnostic_updater::Updater diagnostics_updater_;
};

}  // namespace cpu_temperature_diagnostics