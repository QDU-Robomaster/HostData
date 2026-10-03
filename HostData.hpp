#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 上位机数据接入模块：把上位机发来的云台目标、底盘速度和发射命令汇总为 CMD 的 AI 控制数据 / Host data Module that combines the gimbal target, chassis speed and fire command from the host into AI control data for CMD
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
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

/**
 * @brief 上位机数据接入模块：把上位机发来的云台目标、底盘速度和发射命令汇总为
 *        `CMD::Data`，通过 CMD 的 AI 控制入口提交。
 *        Host data Module that combines the gimbal target, chassis speed and fire command
 *        from the host into `CMD::Data` and submits it through the AI control entry of
 *        CMD.
 */
class HostData
{
 public:
  /**
   * @brief 上位机给出的底盘目标速度。
   *        Chassis target speed from the host.
   */
  struct HostChassisTarget
  {
    float vx;  ///< x 方向速度 Velocity along x
    float vy;  ///< y 方向速度 Velocity along y
    float w;   ///< 旋转角速度 Rotation angular velocity
  };

  /**
   * @brief 上位机给出的发射命令。
   *        Fire command from the host.
   */
  struct LauncherCMD
  {
    bool isfire;  ///< 是否开火 Whether to fire
  };

  /**
   * @brief 上位机给出的云台目标欧拉角及其一阶、二阶导数。
   *        Gimbal target Euler angles from the host with their first and second
   *        derivatives.
   */
  struct HostGimbalTarget
  {
    float rol;       ///< 目标 roll Target roll
    float pit;       ///< 目标 pitch Target pitch
    float yaw;       ///< 目标 yaw Target yaw
    float rol_dot;   ///< roll 一阶导数 First derivative of roll
    float pit_dot;   ///< pitch 一阶导数 First derivative of pitch
    float yaw_dot;   ///< yaw 一阶导数 First derivative of yaw
    float rol_ddot;  ///< roll 二阶导数 Second derivative of roll
    float pit_ddot;  ///< pitch 二阶导数 Second derivative of pitch
    float yaw_ddot;  ///< yaw 二阶导数 Second derivative of yaw
  };

  /**
   * @brief 构造 HostData，创建三个 Topic 并注册回调。
   *        Construct HostData, create the three Topics and register the callbacks.
   *
   * @param cmd CMD 实例，接收 AI 控制数据。
   *            CMD instance that receives the AI control data.
   * @param host_gimbal_topic_name 云台目标 Topic 名称。
   *                               Name of the gimbal target Topic.
   * @param host_chassis_data_topic_name 底盘目标速度 Topic 名称。
   *                                     Name of the chassis target speed Topic.
   * @param host_fire_topic_name 发射命令 Topic 名称。
   *                             Name of the fire command Topic.
   */
  HostData(CMD& cmd, const char* host_gimbal_topic_name = "target_euler",
           const char* host_chassis_data_topic_name = "host_chassis_data",
           const char* host_fire_topic_name = "host_fire_notify")
      : cmd_(&cmd),
        host_gimbal_data_tp_(
            LibXR::Topic::CreateTopic<HostGimbalTarget>(host_gimbal_topic_name)),
        host_chassis_data_tp_(
            LibXR::Topic::CreateTopic<HostChassisTarget>(host_chassis_data_topic_name)),
        host_fire_notify_tp_(LibXR::Topic::CreateTopic<LauncherCMD>(host_fire_topic_name))
  {
    auto euler_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data, const HostGimbalTarget& t)
        {
          host_data->host_euler_ = LibXR::EulerAngle<float>(t.rol, t.pit, t.yaw);
          host_data->host_gyro_ =
              Eigen::Matrix<float, 3, 1>(t.rol_dot, t.pit_dot, t.yaw_dot);
          host_data->host_accl_ =
              Eigen::Matrix<float, 3, 1>(t.rol_ddot, t.pit_ddot, t.yaw_ddot);
          host_data->last_gimbal_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto chassis_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data, const HostChassisTarget& chassis)
        {
          host_data->host_chassis_data_ = chassis;
          host_data->last_chassis_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    auto fire_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, HostData* host_data, const LauncherCMD& fire)
        {
          host_data->host_fire_notify_ = fire;
          host_data->last_fire_time_ = LibXR::Timebase::GetMilliseconds();
          host_data->HostCMD(in_isr);
        },
        this);

    host_gimbal_data_tp_.RegisterCallback(euler_callback);
    host_chassis_data_tp_.RegisterCallback(chassis_callback);
    host_fire_notify_tp_.RegisterCallback(fire_callback);
  }

  /**
   * @brief 用三路最新数据组成一帧 `CMD::Data` 并调用 `cmd.FeedAI()`。
   *        Compose one `CMD::Data` frame from the latest data of the three Topics and
   *        call `cmd.FeedAI()`.
   *
   * @param in_isr 是否在中断上下文。
   *               Whether called from interrupt context.
   */
  void HostCMD(bool in_isr)
  {
    UNUSED(in_isr);
    CMD::Data host_cmd;

    if (host_chassis_data_.vx == 0.0f && host_chassis_data_.vy == 0.0f &&
        host_chassis_data_.w == 0.0f)
    {
      host_cmd.chassis = {0, 0, 0};
      host_cmd.chassis_online = false;
    }
    else
    {
      host_cmd.chassis.x = host_chassis_data_.vx;
      host_cmd.chassis.y = host_chassis_data_.vy;
      host_cmd.chassis.z = host_chassis_data_.w;
      host_cmd.chassis_online = true;
    }

    if (host_euler_.Pitch() == 0.0f && host_euler_.Yaw() == 0.0f)
    {
      host_cmd.gimbal = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      host_cmd.gimbal_online = false;
    }
    else
    {
      host_cmd.gimbal.pit = host_euler_.Pitch();
      host_cmd.gimbal.yaw = host_euler_.Yaw();
      host_cmd.gimbal.pit_dot = host_gyro_.y();
      host_cmd.gimbal.pit_ddot = host_accl_.y();
      host_cmd.gimbal.yaw_dot = host_gyro_.z();
      host_cmd.gimbal.yaw_ddot = host_accl_.z();
      host_cmd.gimbal_online = true;
    }

    host_cmd.launcher.isfire = host_fire_notify_.isfire;

    host_cmd.ctrl_source = CMD::ControlSource::CTRL_SOURCE_AI;
    cmd_->FeedAI(host_cmd);
  }

 private:
  CMD* cmd_;
  HostChassisTarget host_chassis_data_;
  LauncherCMD host_fire_notify_;

  LibXR::EulerAngle<float> host_euler_;
  Eigen::Matrix<float, 3, 1> host_gyro_ = {0, 0, 0};
  Eigen::Matrix<float, 3, 1> host_accl_ = {0, 0, 0};

  LibXR::Topic host_gimbal_data_tp_;
  LibXR::Topic host_chassis_data_tp_;
  LibXR::Topic host_fire_notify_tp_;

  LibXR::MillisecondTimestamp last_chassis_time_ = 0;
  LibXR::MillisecondTimestamp last_gimbal_time_ = 0;
  LibXR::MillisecondTimestamp last_fire_time_ = 0;
};
