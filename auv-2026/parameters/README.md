# ROV parameters 参数文件说明

本目录保存 ROV 程序启动时直接读取的持久化参数。标准 JSON 不支持 `//` 或
`/* ... */` 注释；JSON中的简短 `_comment` 字段仅供阅读，详细中文说明见本文档。

程序必须以项目根目录为工作目录启动，才能通过 `./parameters/...` 找到这些文件。

## control_parameters.json

### controllers

`controllers` 保存十组 PID 控制器参数：

| JSON 名称             | 中文名称 | 控制对象 |
| -----------------| ------------- | --- |
| `pitch_angle`     | 俯仰角度控制器     | 控制俯仰角度 |
| `pitch_gyro`      | 俯仰角速度控制器    | 控制俯仰角速度 |
| `roll_angle`      | 横滚角度控制器   | 控制横滚角度 |
| `roll_gyro`       | 横滚角速度控制器 | 控制横滚角速度 |
| `yaw_angle`       | 偏航角度控制器   | 控制航向角 |
| `yaw_gyro`        | 偏航角速度控制器 | 控制偏航角速度 |
| `depth_position`  | 深度位置控制器   | 保持或改变目标深度 |
| `depth_speed`     | 深度速度控制器  | 控制垂直升降速度 |
| `position_x`      | X 轴位置控制器  | 水平 X 方向位置补偿 |
| `position_y`      | Y 轴位置控制器   | 水平 Y 方向位置补偿 |

每组控制器均包含以下参数：

| JSON 参数 | 中文名称 | 含义及取值 |
| --- | --- | --- |
| `p` | 比例系数 Kp | 当前误差对应的比例输出增益 |
| `i` | 积分系数 Ki | 累计误差对应的积分输出增益 |
| `d` | 微分系数 Kd | 误差变化量对应的微分输出增益 |
| `error_limit_enabled` | 误差限幅开关 | `1` 启用，`0` 关闭 |
| `integral_limit_enabled` | 积分限幅开关 | `1` 启用，`0` 关闭 |
| `integral_separation_enabled` | 积分分离开关 | `1` 启用，`0` 关闭 |
| `error_limit` | 最大误差 | 启用误差限幅后，误差绝对值不能超过该值 |
| `integral_separation_error` | 积分分离误差阈值 | 误差绝对值超过该值时停止累积积分；当前实现中该逻辑仍需实机验证 |
| `integral_limit` | 最大积分值 | 启用积分限幅后，积分累计绝对值不能超过该值 |
| `output_limit` | 最大输出值 | PID 最终输出绝对值不能超过该值 |
| `p_scale` | 比例项缩放系数 | 对 `Kp × 误差` 的结果再次缩放，通常为 `1.0` |
| `i_scale` | 积分项缩放系数 | 对 `Ki × 积分` 的结果再次缩放，通常为 `1.0` |
| `d_scale` | 微分项缩放系数 | 对 `Kd × 误差变化量` 的结果再次缩放，通常为 `1.0` |

配置文件不保存以下运行时数据，它们在程序启动时清零，并在控制过程中动态变化：

| C 结构体成员 | 中文名称 |
| --- | --- |
| `Expect` | 期望值/目标值 |
| `FeedBack` | 传感器反馈值 |
| `Err` | 当前误差 |
| `Last_Err` | 上一次误差 |
| `Integrate` | 当前积分累计值 |
| `Control_OutPut` | 当前控制输出 |
| `Last_Control_OutPut` | 上一次控制输出 |

### devices

`devices` 保存推进器以外的普通 PWM 设备参数。目前只有机械臂：

| JSON 名称 | 中文名称 | 含义 |
| --- | --- | --- |
| `robot_arm` | 机械臂 | 机械臂 PWM 设备配置 |
| `min_pwm` | 最小 PWM 脉宽 | 机械臂允许的最小脉宽，单位为微秒（μs） |
| `max_pwm` | 最大 PWM 脉宽 | 机械臂允许的最大脉宽，单位为微秒（μs） |
| `speed` | PWM 变化步长 | 每个更新周期允许改变的脉宽步长，单位为微秒（μs） |

## propeller_parameters.json

`propeller_parameters` 保存六路推进器配置：

| JSON 名称 | 中文名称 |
| --- | --- |
| `front_left` | 左前推进器 |
| `front_right` | 右前推进器 |
| `center_left` | 左中推进器 |
| `center_right` | 右中推进器 |
| `back_left` | 左后推进器 |
| `back_right` | 右后推进器 |

每路推进器包含以下参数：

| JSON 参数 | 中文名称 | 含义及取值 |
| --- | --- | --- |
| `deadzone_lower` | 死区下限 | 输入位于死区范围内时不产生有效推力 |
| `deadzone_upper` | 死区上限 | 输入位于死区范围内时不产生有效推力 |
| `power_positive` | 正方向动力比例 | 正方向输出缩放比例 |
| `power_negative` | 负方向动力比例 | 负方向输出缩放比例 |
| `reversed` | 方向反转开关 | `true` 反转推进器方向，`false` 保持原方向 |
| `enabled` | 推进器启用开关 | `true` 启用，`false` 禁用 |

修改参数后必须保持合法 JSON 格式。程序启动时若配置文件不存在、无法解析或
缺少必要 PID 字段，将报告错误并停止启动，避免在参数不完整时运行控制任务。
