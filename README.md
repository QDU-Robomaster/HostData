# HostData

上位机数据接入模块：把上位机发来的云台目标、底盘速度和发射命令汇总为 CMD 的 AI 控制数据 / Host data Module that combines the gimbal target, chassis speed and fire command from the host into AI control data for CMD

## 1. 模块作用 / Purpose

构造时，HostData 按构造参数给出的名字创建三个 Topic 并在其上注册回调。上位机链路（例如 `xrobot-org/SharedTopic`）向这些 Topic 发布数据，数据结构与第 3 节的类型一致。

任一 Topic 收到数据时，回调用三路最新数据组成一帧 `CMD::Data`，`ctrl_source` 设为 `CMD::ControlSource::CTRL_SOURCE_AI`，并调用 `cmd.FeedAI()`：

- 底盘：`vx, vy, w` 写入 `chassis.x / y / z`；三者全为 0 时 `chassis_online = false`，否则为 `true`。
- 云台：写入 `pit`、`yaw` 及 `pit_dot`、`pit_ddot`、`yaw_dot`、`yaw_ddot`；pitch 与 yaw 都为 0 时 `gimbal_online = false` 且云台数据清零，否则为 `true`。
- 发射：`launcher.isfire` 取最近一次发射命令。

回调在发布方的上下文中执行。

Upon construction, HostData creates three Topics with the names given by the constructor parameters and registers a callback on each. The host link (for example `xrobot-org/SharedTopic`) publishes data to these Topics, with data structures matching the types in section 3.

When any Topic receives data, the callback composes one `CMD::Data` frame from the latest data of the three Topics, sets `ctrl_source` to `CMD::ControlSource::CTRL_SOURCE_AI`, and calls `cmd.FeedAI()`:

- Chassis: `vx, vy, w` are written to `chassis.x / y / z`; `chassis_online = false` when all three are 0, otherwise `true`.
- Gimbal: `pit`, `yaw`, `pit_dot`, `pit_ddot`, `yaw_dot` and `yaw_ddot` are written; `gimbal_online = false` and the gimbal data is cleared when pitch and yaw are both 0, otherwise `true`.
- Fire: `launcher.isfire` takes the latest fire command.

The callbacks run in the context of the publisher.

## 2. 构造接口 / Constructor

```cpp
HostData(CMD& cmd,
         const char* host_gimbal_topic_name = "target_euler",
         const char* host_chassis_data_topic_name = "host_chassis_data",
         const char* host_fire_topic_name = "host_fire_notify");
```

依赖：

- `cmd`：`CMD` 实例，接收 AI 控制数据。

配置参数：

- `host_gimbal_topic_name`：云台目标 Topic 名称，默认 `"target_euler"`。
- `host_chassis_data_topic_name`：底盘速度 Topic 名称，默认 `"host_chassis_data"`。
- `host_fire_topic_name`：发射命令 Topic 名称，默认 `"host_fire_notify"`。

Dependencies:

- `cmd`: the `CMD` instance that receives the AI control data.

Configuration parameters:

- `host_gimbal_topic_name`: name of the gimbal target Topic, default `"target_euler"`.
- `host_chassis_data_topic_name`: name of the chassis speed Topic, default `"host_chassis_data"`.
- `host_fire_topic_name`: name of the fire command Topic, default `"host_fire_notify"`.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `host_gimbal_topic_name`（默认 `target_euler`） | 创建并订阅 | `HostData::HostGimbalTarget` | 目标 `rol, pit, yaw` 及其一阶（`*_dot`）、二阶（`*_ddot`）导数 |
| `host_chassis_data_topic_name`（默认 `host_chassis_data`） | 创建并订阅 | `HostData::HostChassisTarget` | 底盘速度 `vx, vy, w` |
| `host_fire_topic_name`（默认 `host_fire_notify`） | 创建并订阅 | `HostData::LauncherCMD` | 发射命令 `isfire` |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `host_gimbal_topic_name` (default `target_euler`) | Create and subscribe | `HostData::HostGimbalTarget` | Target `rol, pit, yaw` with their first (`*_dot`) and second (`*_ddot`) derivatives |
| `host_chassis_data_topic_name` (default `host_chassis_data`) | Create and subscribe | `HostData::HostChassisTarget` | Chassis speed `vx, vy, w` |
| `host_fire_topic_name` (default `host_fire_notify`) | Create and subscribe | `HostData::LauncherCMD` | Fire command `isfire` |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/HostData` 写入的实例，`cmd` 填写为 `QDU-Robomaster/CMD` 实例的 id，该实例在 `modules:` 中排在本实例之前。Topic 名称与上位机链路使用的名称一致；示例中的 `fire_notify` 与 Dart 订阅的发射命令 Topic 名称相同。

An instance written by `xrobot instance add QDU-Robomaster/HostData`, with `cmd` set to the id of the `QDU-Robomaster/CMD` instance, which is listed before this instance in `modules:`. The Topic names match the names used by the host link; `fire_notify` in the example is the fire command Topic name that Dart subscribes to.

```yaml
modules:
  - module: QDU-Robomaster/HostData
    id: hostdata
    args:
      - cmd: cmd
      - host_gimbal_topic_name: "target_euler"
      - host_chassis_data_topic_name: "chassis_data"
      - host_fire_topic_name: "fire_notify"
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CMD`：`CMD::Data` 类型与 `FeedAI()` 入口。
- LibXR。

硬件：无。

Dependencies:

- `QDU-Robomaster/CMD`: the `CMD::Data` type and the `FeedAI()` entry.
- LibXR.

Hardware: none.
