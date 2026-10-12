# AUV/ROV Master 下位机

本项目运行在 NanoPi 上，负责接收上位机控制命令、控制推进器和机械臂、保存参数，并向上位机返回设备状态。

当前主要功能：

- 连接上位机并接收手柄运动控制
- ROV模式保持原有开环手柄控制
- AUV模式提供相对深度和相对航向两个单环PID控制
- 设置、读取和保存PID参数，读取PID实时曲线数据
- 设置和保存推进器参数
- RGB 状态灯：蓝色等待连接、绿色正常运行、红色故障
- 网络摄像头画面由上位机单独拉取，不由本程序显示

## PCA9685通道配置

NanoPi与PCA9685的连接、六路推进器通道和机械臂通道统一在
`user/config.h`中配置。PCA9685物理输出通道使用0～15的编号。

六路推进器对应以下宏：

```c
#define PROPELLER_RIGHT_UP_CHANNEL     5
#define PROPELLER_RIGHT_MIDDLE_CHANNEL 4
#define PROPELLER_RIGHT_DOWN_CHANNEL   3
#define PROPELLER_LEFT_UP_CHANNEL      2
#define PROPELLER_LEFT_MIDDLE_CHANNEL  6
#define PROPELLER_LEFT_DOWN_CHANNEL    7
```

通道超出0～15、推进器通道重复，或者机械臂与推进器占用同一通道时，
程序会在启动阶段记录错误并拒绝启动PWM输出。

## 编译

首次在 NanoPi 上部署时：

```sh
chmod +x build.sh compile.sh
./build.sh
```

以后只修改本项目 C 代码时：

```sh
make
```

生成的程序位于 `build/auvmaster`。在项目根目录手动运行：

```sh
sudo ./build/auvmaster
```

项目会在主程序构建过程中直接编译自带的`lib/libev-4.22`源码，
不需要另外下载、配置或安装libev。

手动运行时要位于项目根目录，因为程序从 `./data` 读取参数文件。

## 摄像头画面

本项目不负责启动摄像头、传输视频或显示画面。网络摄像头由上位机使用摄像头地址手动拉流，画面显示在上位机的视频界面中，与下位机的 JSON-RPC 控制连接相互独立。

## 参数文件

- `parameters/control_parameters.json`：全部 PID 与普通 PWM 设备参数
- `parameters/propeller_parameters.json`：各推进器死区、启用、反转和动力比例

## JSON-RPC 方法

- `status`：连接检测并获取设备状态
- `move`：手柄运动控制
- `gripper`：机械臂控制
- `set_auv_target`：设置相对深度和相对航向目标
- `pid_runtime_status`：读取PID实时目标、反馈、误差和输出
- `get_pid_params`：读取PID参数
- `set_propeller_params`：修改推进器参数
- `set_pid`：修改 PID 参数
- `load_params`：从文件重新读取参数
- `save_propeller_params`：保存推进器参数
- `save_pid`：保存 PID 参数

代码中暂时未调用的 `wk2132` 驱动和滤波文件为后续硬件、传感器处理保留，并带有“当前未使用”的注释。
