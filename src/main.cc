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

#include <stdexcept>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include "cpu_temperature_diagnostics/cpu_temperature_diagnostic.hh"

namespace
{
std::vector<
    std::unique_ptr<cpu_temperature_diagnostics::CpuTemperatureDiagnostic>>
create_diagnostics(std::string& prefix)
{
    auto chips = cpu_temperature_diagnostics::sensors_chip_factory::
        get_chips_with_prefix(prefix);

    std::vector<
        std::unique_ptr<cpu_temperature_diagnostics::CpuTemperatureDiagnostic>>
        diag_vec;

    for (auto& chip : chips)
    {
        auto new_diag = std::make_unique<
            cpu_temperature_diagnostics::CpuTemperatureDiagnostic>(chip);
        diag_vec.push_back(std::move(new_diag));
    }

    return diag_vec;
}

void diagnose(
    std::vector<
        std::unique_ptr<cpu_temperature_diagnostics::CpuTemperatureDiagnostic>>&
        diag_vec)
{
    for (auto& diagnostic : diag_vec)
    {
        diagnostic->emit_diagnostics();
    }
}

void configure_diagnostics(
    ros::NodeHandle& node_handle,
    std::vector<
        std::unique_ptr<cpu_temperature_diagnostics::CpuTemperatureDiagnostic>>&
        diag_vec)
{
    if (int max_temp_override;
        node_handle.getParam("max_temp_override", max_temp_override))
    {
        ROS_WARN_STREAM("Maximum temperature set to " +
                        std::to_string(max_temp_override));
        for (auto& diag : diag_vec)
        {
            diag->max_temp_override(max_temp_override);
        }
    }

    if (int critical_temp_override;
        node_handle.getParam("critical_temp_override", critical_temp_override))
    {
        ROS_WARN_STREAM("Critical temperature set to " +
                        std::to_string(critical_temp_override));
        for (auto& diag : diag_vec)
        {
            diag->critical_temp_override(critical_temp_override);
        }
    }
}

}  // namespace
int main(int argc, char* argv[])
{
    sensors_init(nullptr);
    ros::init(argc, argv, "sensors_diagnostics");
    ros::NodeHandle node_handle("~");
    std::string prefix;

    if (!node_handle.getParam("prefix", prefix))
    {
        std::string err = "prefix parameter is missing";
        ROS_ERROR_STREAM(err);
        exit(1);
    }

    auto diag_vec = create_diagnostics(prefix);
    configure_diagnostics(node_handle, diag_vec);

    auto current_timer = node_handle.createTimer(
        ros::Duration(1),
        [&diag_vec](const ros::TimerEvent&) { diagnose(diag_vec); });
    ros::spin();
    sensors_cleanup();
    return 0;
}
