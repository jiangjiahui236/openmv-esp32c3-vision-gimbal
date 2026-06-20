# OpenMV ESP32-C3 Vision Gimbal

中文名：OpenMV ESP32-C3 视觉追踪云台

这是一个基于 OpenMV、ESP32-C3 和 Arduino Nano 的二自由度视觉追踪云台项目。OpenMV 负责识别人脸或红色圆球，ESP32-C3 提供 Wi-Fi 热点网页和参数下发，Arduino Nano 作为 I2C 从机负责舵机平滑控制。

## 功能特点

- OpenMV H7/H7 R2 视觉识别
  - 支持固件自带 Haar cascade 人脸识别
  - 支持 LAB 颜色阈值红色圆球识别
- ESP32-C3 网页控制
  - 自建 Wi-Fi 热点，浏览器打开即可控制
  - 支持摇杆、角度滑条、自动/手动模式切换
  - 支持实时调节增益、死区、步进、控制周期和滤波
  - 参数可写入 ESP32-C3 Flash，重启后自动加载
  - 可选接入 1.3 寸 I2C OLED 表情屏
- Arduino Nano 舵机控制
  - I2C 从机地址为 `0x12`
  - 接收 yaw/pitch 角度并平滑驱动两个 180 度舵机

## 目录结构

```text
esp32c3_web_gimbal/          ESP32-C3 热点网页、OpenMV 串口、Nano I2C、可选 OLED
nano_i2c_gimbal_slave/       Arduino Nano I2C 舵机从机程序
nano_gimbal_test/            Arduino Nano 单机舵机测试程序
openmv_red_ball_tracker/     OpenMV 人脸/红球追踪脚本
```

## 硬件清单

- ESP32-C3 开发板
- Arduino Nano
- OpenMV H7 或 OpenMV H7 R2
- 2 个 180 度舵机
- 外部 5V 舵机电源
- 可选：1.3 寸 128x64 I2C OLED，常见控制器为 SH1106，地址为 `0x3C`

所有模块必须共地。舵机不要直接从 ESP32-C3 或 Arduino Nano 板载 5V 取电，建议使用独立 5V 电源给舵机供电。

## 接线

### Nano 与舵机

```text
Nano D3 -> yaw 舵机信号线
Nano D5 -> pitch 舵机信号线
舵机 VCC -> 外部 5V 舵机电源
舵机 GND -> 外部电源 GND
Nano GND -> 外部电源 GND
```

### ESP32-C3 与 Nano

```text
ESP32-C3 GPIO4 / SDA -> Nano A4
ESP32-C3 GPIO5 / SCL -> Nano A5
ESP32-C3 GND         -> Nano GND
```

Arduino Nano 通常是 5V 逻辑，ESP32-C3 是 3.3V 逻辑。实际使用时建议在 I2C 总线上加双向电平转换模块，避免 ESP32-C3 GPIO 承受 5V。

### OpenMV 与 ESP32-C3

```text
OpenMV P4 / UART3 TX -> ESP32-C3 GPIO1 / RX
OpenMV P5 / UART3 RX -> ESP32-C3 GPIO0 / TX
OpenMV GND           -> ESP32-C3 GND
```

OpenMV 与 ESP32-C3 都是 3.3V 串口，可直接连接，但 TX/RX 需要交叉接线。

### 可选 OLED

OLED 与 Nano 共用 ESP32-C3 的 I2C 总线：

```text
OLED VCC -> 3.3V
OLED GND -> ESP32-C3 GND
OLED SDA -> ESP32-C3 GPIO4 / SDA
OLED SCL -> ESP32-C3 GPIO5 / SCL
```

默认配置在 `esp32c3_web_gimbal/esp32c3_web_gimbal.ino` 中：

```cpp
const byte OLED_I2C_ADDRESS = 0x3C;
const bool OLED_CONTROLLER_SH1106 = true;
const bool OLED_ROTATE_180 = true;
```

如果你的 OLED 是 SSD1306，把 `OLED_CONTROLLER_SH1106` 改为 `false`；如果地址是 `0x3D`，把 `OLED_I2C_ADDRESS` 改为 `0x3D`。

## 默认网络

ESP32-C3 程序会创建 Wi-Fi 热点：

```text
SSID: Gimbal-ESP32C3
Password: change-me
Web UI: http://192.168.4.1/
```

开源版本中的密码是占位值。正式使用前请修改 `esp32c3_web_gimbal/esp32c3_web_gimbal.ino` 里的：

```cpp
const char *AP_SSID = "Gimbal-ESP32C3";
const char *AP_PASSWORD = "change-me";
```

## 烧录与运行

1. 将 `nano_i2c_gimbal_slave/nano_i2c_gimbal_slave.ino` 烧录到 Arduino Nano。
2. 将 `esp32c3_web_gimbal/esp32c3_web_gimbal.ino` 烧录到 ESP32-C3。
3. 将 `openmv_red_ball_tracker/main.py` 复制到 OpenMV 并运行。
4. 手机或电脑连接 ESP32-C3 热点，浏览器打开 `http://192.168.4.1/`。
5. 在网页中切换手动控制、自动追踪和追踪目标，并根据现场效果调参。

## 通信协议

### ESP32-C3 到 Nano

I2C 地址：`0x12`

每次发送 2 字节：

```text
byte0 = yaw angle
byte1 = pitch angle
```

### OpenMV 到 ESP32-C3

UART 波特率：`57600`

OpenMV 发送当前角度：

```text
yaw,pitch\n
```

示例：

```text
90,90
```

### ESP32-C3 到 OpenMV

心跳：

```text
OK\n
```

动态调参：

```text
CFG,ykp,pkp,dz,step,ms,alpha,target\n
```

参数含义：

- `ykp`：yaw 追踪增益
- `pkp`：pitch 追踪增益
- `dz`：死区像素
- `step`：每次控制最大角度变化
- `ms`：控制周期，单位 ms
- `alpha`：目标点滤波系数
- `target`：追踪目标，`0` 为人脸，`1` 为红圆

## 调参建议

推荐从下面这组参数开始：

```text
Yaw Kp:   0.020
Pitch Kp: 0.020
步进:     2.0
周期:     50 ms
死区:     12 px
滤波:     0.35
```

常见现象和处理方式：

- 追踪太慢：先增大步进，再适当增大 `Yaw Kp` 和 `Pitch Kp`
- 云台开始绕目标画圈：降低 `Yaw Kp` 和 `Pitch Kp`
- 接近中心仍抖动：增大死区
- 识别点跳动导致舵机抖动：降低滤波响应速度，也就是把 `alpha` 调小
- 跟随延迟明显：减小控制周期，例如从 80 ms 改到 50 ms

如果云台朝远离目标的方向运动，修改 OpenMV 脚本中的方向符号：

```python
YAW_SIGN = -1
PITCH_SIGN = 1
```

哪个轴方向反了，就把对应符号取反。

## OpenMV 人脸识别说明

人脸识别依赖 OpenMV 固件自带的 Haar cascade 文件：

```text
/rom/haarcascade_frontalface.cascade
```

脚本会依次尝试 `/rom/haarcascade_frontalface.cascade`、`frontalface`、`frontalface.cascade` 三种路径。如果都找不到，不会直接崩溃，而是临时退回红圆模式。遇到这种情况，可以在 OpenMV IDE 中执行 `Tools -> Reset OpenMV Cam`，或更新/重刷固件以恢复 ROMFS 文件。

## 开源发布说明

本目录用于发布到 GitHub，已排除课程报告、照片、视频、本地开发记录和机器相关文件。

发布前建议在本目录运行一次扫描：

```powershell
rg -n -i "(password|passwd|token|secret|api[_-]?key|ssid|wifi|邮箱|电话|学号|姓名|C:\\\\Users|PRIVATE|BEGIN RSA|OPENAI|github|ghp_|sk-)" .
```

预期会命中 `AP_SSID`、`AP_PASSWORD`、README 网络说明和 `OPEN_SOURCE_CHECK.md`。其中 `change-me` 是给使用者替换的占位密码。

## 许可证

本项目使用 MIT License，详见 [LICENSE](LICENSE)。
