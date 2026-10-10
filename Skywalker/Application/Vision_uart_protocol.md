# 视觉上位机与云台 UART 协议

视觉链路在 `app_standard.c` 中由 `kStandardVisionEnabled = 1U` 开启。
USART10 为 **921600 baud、8N1、无流控**：PE3/TX 接上位机 RX，PE2/RX 接上位机 TX，并共地，使用 3.3 V TTL 电平。

## 帧格式

沿用 `app_protocol.h` 的 AB 二进制协议，1 字节对齐，没有长度字段或帧尾。
`float` 为 IEEE 754 的 32 位浮点数，所有多字节字段按小端发送。
帧头是 ASCII `AB`，即 `0x41 0x42`，不是一个 `0xAB` 字节。
CRC16 覆盖帧头和全部数据，不含 CRC 自身；初值 `0xFFFF`，反射多项式 `0x8408`（正向 `0x1021`），无末尾异或，低字节先发送。`123456789` 的校验值为 `0x6F91`。

### 上位机 → 云台：29 字节

| 字节偏移 | 类型 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | uint8[2] | head | `AB` |
| 2 | uint8 | mode | 0：释放视觉控制；1：控制云台；2：控制云台并请求开火 |
| 3 | float | yaw | 相对 DM 电机零点的目标角度，rad |
| 7 | float | yaw_velocity | 目标角速度，rad/s；当前 DM 位置速度模式不使用此字段 |
| 11 | float | yaw_acceleration | 目标角加速度，rad/s²；当前保留 |
| 15 | float | pitch | 相对云台中心的目标角度，rad，抬头为正 |
| 19 | float | pitch_velocity | 目标角速度，rad/s，抬头为正；用于 Pitch 滑模控制的速度参考 |
| 23 | float | pitch_acceleration | 目标角加速度，rad/s²；当前保留 |
| 27 | uint16 | crc16 | 前 27 字节的 CRC16 |

### 云台 → 上位机：43 字节

| 字节偏移 | 类型 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | uint8[2] | head | `AB` |
| 2 | uint8 | mode | 0：云台关闭或安全停机；1：云台可控制、自瞄；当前不选择小符/大符 |
| 3 | float[4] | q | wxyz；当前固定 `[1, 0, 0, 0]`，尚未接入云台姿态估计 |
| 19 | float | yaw | DM 电机反馈角度，rad，与下发 yaw 使用同一零点 |
| 23 | float | yaw_velocity | DM 电机反馈角速度，rad/s |
| 27 | float | pitch | 中心偏置减去 GM6020 反馈角度，rad，抬头为正；未收反馈为 0 |
| 31 | float | pitch_velocity | GM6020 反馈角速度取负，rad/s，抬头为正 |
| 35 | float | bullet_speed | 弹速，m/s；未启用裁判系统时为 0 |
| 39 | uint16 | bullet_count | 现有累计发弹数变量；本次接入不新增计数逻辑 |
| 41 | uint16 | crc16 | 前 41 字节的 CRC16 |

Manager 任务每 10 ms 尝试回传一次，UART 忙时跳过当次发送。

## 控制行为

- 收到 CRC 正确、mode 合法且所有浮点字段有限的完整帧后，云台任务每 2 ms 读取一致的指令快照。接收支持半包、粘包和坏帧后重新同步。
- `mode=1/2` 且最新有效指令不超过 100 ms 时，视觉优先控制 Yaw/Pitch；未收到指令、`mode=0` 或超时则恢复现有遥控控制。上位机应持续发送指令，建议周期 10–20 ms。
- Yaw 目标限幅为 `[-π, π]`，继续使用现有 DM 位置速度模式，运动速度上限保持 18 rad/s。`yaw_velocity` 和两个加速度字段保留在协议中，目前不参与控制。
- 视觉 Pitch 约定抬头为正，实机抬头时编码器减小。电机目标角度为 `2070 / 8192 × 2π - pitch`，电机速度参考为 `-pitch_velocity`，回传角度、角速度也采用抬头为正的约定。`pitch=+0.1 rad` 表示抬头约 5.73°，对应编码器约 1940；`pitch=-0.1 rad` 表示低头约 5.73°，对应编码器约 2200。
- Pitch 目标限幅为编码器 `[1422, 2759]` 对应的角度范围，视觉角度范围约为 `[-0.5285, 0.4970] rad`。速度参考限幅为 ±6 rad/s，到边界时清除继续向外的速度参考，沿用现有滑模控制、电流限幅和重力补偿。
- 从视觉切回遥控时，控制时间步长不会累积视觉控制期间的时间；视觉/遥控切换时清除 Pitch 控制积分。
- 遥控Pitch调参时，`kStandardPitchRcTrajectoryEnabled = 0U` 直接采用摇杆目标角度，速度参考为0；设为 `1U` 恢复遥控平滑轨迹。视觉分支继续直接采用上位机目标。
- 遥控器必须在线，原有遥控离线停机逻辑仍生效。视觉指令不绕过安全停机。
- `mode=2` 的视觉开火请求也检查有效指令和 100 ms 超时；遥控开火操作继续独立生效。仅调试云台时发送 `mode=1`。
- USART1 的 VOFA+ FireWater 遥测独立输出Pitch目标/实际位置（度）和输出/反馈电流（A），共4个通道；顺序见 `FireWater_telemetry.md`。

## Python 发包示例

以下函数只负责生成帧；通过配置为 921600、8N1 的串口发送返回值。

```python
import struct

def make_vision_frame(yaw, pitch, pitch_velocity=0.0, mode=1):
    payload = struct.pack("<2sB6f", b"AB", mode,
                          yaw, 0.0, 0.0, pitch, pitch_velocity, 0.0)
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return payload + struct.pack("<H", crc)
```

## 本机协议测试

使用本机 C 编译器（不使用 ARM 交叉编译器）执行：

```powershell
cmake -S tests/vision_protocol -B build/vision-protocol-tests -G Ninja
cmake --build build/vision-protocol-tests
ctest --test-dir build/vision-protocol-tests --output-on-failure
```

测试覆盖逐字节/任意分包点接收、粘包、错误帧头重新同步、CRC 错误、非法模式与 NaN/Inf、控制释放、超时与时钟回绕、非 1 kHz 时钟、回传布局和 CRC。固件编译用 `cmake --build --preset Debug`；实际运动方向和机械限位需在实机确认。
