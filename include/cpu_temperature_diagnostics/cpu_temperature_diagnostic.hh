#pragma once

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "cpu_temperature_diagnostics/sensor_chip.hh"

namespace cpu_temperature_diagnostics
{
class CpuTemperatureDiagnostic
{
  public:
    CpuTemperatureDiagnostic(SensorChip& chip);
    void emit_diagnostics();
    void critical_temp_override(int critical_temp);
    void max_temp_override(int max_temp);

  private:
    void diagnostics_callback(
        diagnostic_updater::DiagnosticStatusWrapper& stat) const;

    SensorChip chip_;
    diagnostic_updater::Updater diagnostics_updater_;
};

}  // namespace cpu_temperature_diagnostics