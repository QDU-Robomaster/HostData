# HostData

上位机数据接入模块：把上位机发来的云台目标、底盘速度和发射命令汇总为 `CMD::Data`，
以 AI 控制源喂给 CMD 模块。

- 构造时创建三个 Topic（名字由构造参数给出）并在其上注册回调：

  | 参数 | 默认名字 | 类型 | 内容 |
  | --- | --- | --- | --- |
  | `host_gimbal_topic_name` | `target_euler` | `HostData::HostGimbalTarget` | 目标 `rol, pit, yaw` 及其一阶（`*_dot`）、二阶（`*_ddot`）导数 |
  | `host_chassis_data_topic_name` | `host_chassis_data` | `HostData::HostChassisTarget` | 底盘速度 `vx, vy, w` |
  | `host_fire_topic_name` | `host_fire_notify` | `HostData::LauncherCMD` | 发射命令 `isfire` |

  上位机链路（例如 `xrobot-org/SharedTopic`）向这些 Topic 发布数据，数据结构必须与上表一致。
- 任一 Topic 收到数据时，回调用三路最新数据组成一帧 `CMD::Data`，`ctrl_source` 设为
  `CMD::ControlSource::CTRL_SOURCE_AI`，调用 `cmd.FeedAI()`：
  - 底盘：`vx, vy, w` 写入 `chassis.x / y / z`；三者全为 0 时 `chassis_online = false`，否则为 `true`。
  - 云台：写入 `pit`、`yaw` 及 `pit_dot`、`pit_ddot`、`yaw_dot`、`yaw_ddot`（roll 不使用）；
    pitch 与 yaw 都为 0 时 `gimbal_online = false` 且云台数据清零，否则为 `true`。
  - 发射：`launcher.isfire` 取最近一次发射命令。
- 回调在发布方的上下文中执行，模块没有自己的线程。

## 依赖

- `QDU-Robomaster/CMD`：`CMD::Data` 类型与 `FeedAI()` 入口。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
HostData(CMD& cmd,
         const char* host_gimbal_topic_name = "target_euler",
         const char* host_chassis_data_topic_name = "host_chassis_data",
         const char* host_fire_topic_name = "host_fire_notify");
```

依赖：

- `cmd`：`CMD` 实例，接收 AI 数据。

配置：

- `host_gimbal_topic_name`：云台目标 Topic 名，默认 `"target_euler"`。
- `host_chassis_data_topic_name`：底盘速度 Topic 名，默认 `"host_chassis_data"`。
- `host_fire_topic_name`：发射命令 Topic 名，默认 `"host_fire_notify"`。

## 使用

```sh
xrobot module add QDU-Robomaster/HostData
xrobot setup
xrobot instance add QDU-Robomaster/HostData
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `cmd` 填为 CMD 实例的 id：

```yaml
modules:
  - module: QDU-Robomaster/HostData
    id: hostdata_0
    args:
      - cmd: cmd
      - host_gimbal_topic_name: '"target_euler"'
      - host_chassis_data_topic_name: '"host_chassis_data"'
      - host_fire_topic_name: '"host_fire_notify"'
```

`cmd` 是 `QDU-Robomaster/CMD` 实例的 id，须在本实例之前列出。本例没有需要 BSP 用 `XR_REGISTER`
注册的对象。Topic 名是 C++ 字符串字面量，需与上位机链路使用的名字一致。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/HostData`
（在 BSP 中）打印当前的构造函数。
