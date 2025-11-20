#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - host_euler_topic_name: "target_eulr"
  - host_chassis_data_topic_name: "host_chassis_data"
  - host_fire_topic_name: "host_fire_notify"
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <sys/_intsup.h>

#include "CMD.hpp"
#include "app_framework.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "transform.hpp"

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

  enum class HostFireNotify : uint8_t {
    FIRE_STOP = 0,
    FIRE_START,
  };

  HostData(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           CMD& cmd, const char* host_euler_topic_name,
           const char* host_chassis_data_topic_name,
           const char* host_fire_topic_name)
      : cmd_(&cmd),
        host_euler_data_tp_(
            LibXR::Topic::CreateTopic<HostGimbalEuler>(host_euler_topic_name)),
        host_chassis_data_tp_(LibXR::Topic::CreateTopic<HostChassisTarget>(
            host_chassis_data_topic_name)),
        host_fire_notify_tp_(
            LibXR::Topic::CreateTopic<HostFireNotify>(host_fire_topic_name)),
        ai_cmd_tp_("ai_cmd", sizeof(CMD::Data)) {
    UNUSED(hw);
    UNUSED(app);

    cmd_->RegisterController<CMD::Data>(ai_cmd_tp_);

    auto euler_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_euler_data_,
                                  raw_data.addr_, sizeof(HostGimbalEuler));
        },
        this);

    auto chassis_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_chassis_data_,
                                  raw_data.addr_, sizeof(HostChassisTarget));
          host_data->HostCMD();
        },
        this);

    auto fire_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_fire_notify_,raw_data.addr_, sizeof(HostFireNotify));
        },
        this);
    host_euler_data_tp_.RegisterCallback(euler_callback);
    host_chassis_data_tp_.RegisterCallback(chassis_callback);
    host_fire_notify_tp_.RegisterCallback(fire_callback);
  }

  void HostCMD() {
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
  HostFireNotify host_fire_notify_;

  LibXR::Topic host_euler_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic host_fire_notify_tp_;
  LibXR::Topic ai_cmd_tp_;
};
