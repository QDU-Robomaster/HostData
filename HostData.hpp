#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - host_euler_topic_name: "target_eulr"
  - host_chassis_data_topic_name: "host_chassis_data"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <sys/_intsup.h>

#include "CMD.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "message.hpp"

class HostData : public LibXR::Application {
 public:
  struct HostGimbalEuler {
    float pitch;
    float roll;
    float yaw;
  };

  struct HostChassisTarget {
    float vx;
    float vy;
    float w;
  };

  HostData(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           CMD& cmd, const char* host_euler_topic_name,
           const char* host_chassis_data_topic_name)
      : cmd_(&cmd),
        host_euler_data_tp_(LibXR::Topic::CreateTopic<HostGimbalEuler>(
            host_euler_topic_name)),
        host_chassis_data_tp_(LibXR::Topic::CreateTopic<HostChassisTarget>(
            host_chassis_data_topic_name)),
        ai_cmd_tp_("ai_cmd",sizeof(CMD::Data))
  {
    UNUSED(hw);
    UNUSED(app);

    cmd_->RegisterController<CMD::Data>(ai_cmd_tp_);

    auto euler_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          HostGimbalEuler& euler =
              *static_cast<HostGimbalEuler*>(raw_data.addr_);
          host_data->host_euler_data_ = euler;
          host_data->HostCMD();
        },
        this);

    auto chassis_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          HostChassisTarget& chassis =
              *static_cast<HostChassisTarget*>(raw_data.addr_);
          host_data->host_chassis_data_ = chassis;
          host_data->HostCMD();
        },
        this);

    host_euler_data_tp_.RegisterCallback(euler_callback);
    host_chassis_data_tp_.RegisterCallback(chassis_callback);
  }

  void HostCMD(){
    CMD::Data host_cmd;

    host_cmd.chassis.x = host_chassis_data_.vx;
    host_cmd.chassis.y = host_chassis_data_.vy;
    host_cmd.chassis.z = host_chassis_data_.w;

    host_cmd.gimbal.pit = host_euler_data_.pitch;
    host_cmd.gimbal.yaw = host_euler_data_.yaw;

    host_cmd.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    ai_cmd_tp_.Publish(host_cmd);
  }

  void OnMonitor() override {}

 private:
  CMD* cmd_;
  HostChassisTarget host_chassis_data_;
  HostGimbalEuler host_euler_data_;

  LibXR::Topic host_euler_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic ai_cmd_tp_;
};
