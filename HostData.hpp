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

#include "CMD.hpp"
#include "app_framework.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

class HostData : public LibXR::Application {
 public:
  static constexpr uint32_t DATA_TIMEOUT_MS = 200;

  struct HostChassisTarget {
    float vx;
    float vy;
    float w;
  };

  struct LauncherCMD {
    bool isfire;
  };

  HostData(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           CMD& cmd, const char* host_euler_topic_name,
           const char* host_chassis_data_topic_name,
           const char* host_fire_topic_name)
      : cmd_(&cmd),
        host_euler_data_tp_(LibXR::Topic::CreateTopic<LibXR::EulerAngle<float>>(
            host_euler_topic_name)),
        host_chassis_data_tp_(LibXR::Topic::CreateTopic<HostChassisTarget>(
            host_chassis_data_topic_name)),
        host_fire_notify_tp_(
            LibXR::Topic::CreateTopic<LauncherCMD>(host_fire_topic_name)),
        ai_cmd_tp_("ai_cmd", sizeof(CMD::Data)) {
    UNUSED(hw);

    cmd_->RegisterController<CMD::Data>(ai_cmd_tp_);

    auto euler_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_euler_, raw_data.addr_,
                                  sizeof(host_data->host_euler_));
          host_data->last_gimbal_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto chassis_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_chassis_data_,
                                  raw_data.addr_, sizeof(HostChassisTarget));
          host_data->last_chassis_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto fire_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, HostData* host_data, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          LibXR::Memory::FastCopy(&host_data->host_fire_notify_, raw_data.addr_,
                                  sizeof(LauncherCMD));
          host_data->last_fire_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);
    host_euler_data_tp_.RegisterCallback(euler_callback);
    host_chassis_data_tp_.RegisterCallback(chassis_callback);
    host_fire_notify_tp_.RegisterCallback(fire_callback);
    app.Register(*this);
  }
  void HostCMD(bool in_isr) {
    CMD::Data host_cmd;

    if (host_chassis_data_.vx == 0.0f && host_chassis_data_.vy == 0.0f &&
        host_chassis_data_.w == 0.0f) {
      host_cmd.chassis = {0, 0, 0};
      host_cmd.chassis_online = false;
    } else {
      host_cmd.chassis.x = host_chassis_data_.vx;
      host_cmd.chassis.y = host_chassis_data_.vy;
      host_cmd.chassis.z = host_chassis_data_.w;
      host_cmd.chassis_online = true;
    }

    if (host_euler_.Pitch() == 0.0f && host_euler_.Yaw()  == 0.0f) {
      host_cmd.gimbal = {0, 0, 0};
      host_cmd.gimbal_online = false;
    } else {
      host_cmd.gimbal.pit = host_euler_.Pitch();
      host_cmd.gimbal.yaw = host_euler_.Yaw();
      host_cmd.gimbal_online = true;
    }

    host_cmd.launcher.isfire = host_fire_notify_.isfire;

    host_cmd.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    ai_cmd_tp_.PublishFromCallback(host_cmd, in_isr);
  }

  void OnMonitor() override {}

 private:
  CMD* cmd_;
  HostChassisTarget host_chassis_data_;
  LauncherCMD host_fire_notify_;

  LibXR::EulerAngle<float> host_euler_;

  LibXR::Topic host_euler_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic host_fire_notify_tp_;
  LibXR::Topic ai_cmd_tp_;

  LibXR::MillisecondTimestamp last_chassis_time_ = 0;
  LibXR::MillisecondTimestamp last_gimbal_time_ = 0;
  LibXR::MillisecondTimestamp last_fire_time_ = 0;
};
