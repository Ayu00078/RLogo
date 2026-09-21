# RLogo / PowerRune

RLogo 是一个由 ESP32 多板协同工作的能量机关系统。当前总仓库包含四个固件工程和一个共享协议组件：

| 目录 | 固件角色 | 芯片 | 作用 |
| --- | --- | --- | --- |
| `RLogo_Controller` | Controller | ESP32-S3 | 通过 USB-JTAG 虚拟串口接收命令，并通过 Wi-Fi/TCP 转发给 Master |
| `RLogo_Master` | Master / RLogo | ESP32-C3 | 管理设备上线、游戏流程、Armour 击中判定和灯效 |
| `RLogo_Armour` | Armour | ESP32-S3 | 接收压力传感器 UART，完成第一次击中检测并上报 peak |
| `RLogo_Motor` | Motor | ESP32-C3 | 接收 Master 指令并控制电机 |
| `components/pr_common` | 共享组件 | — | 共享 TCP 协议、数据结构、配置和事件定义 |

## 系统连接

```text
Controller USB-JTAG 虚拟串口
          │
          │ Wi-Fi: PowerRune / TCP 192.168.4.1:8080
          ▼
       Master
       ├── TCP ── Armour 1~5
       └── TCP ── Motor
```

Controller 的地址固定为 `0xFE`，Armour 地址由 `CONFIG_ARMOUR_ID` 决定：

| `CONFIG_ARMOUR_ID` | TCP 协议地址 |
| ---: | ---: |
| 1 | 0 |
| 2 | 1 |
| 3 | 2 |
| 4 | 3 |
| 5 | 4 |

Master 支持 Armour 数量为 0~5 块。Motor 在线后即可进入 READY；缺少 Armour 时会通过 Controller 输出缺少数量的 warning。

## 开发环境

- ESP-IDF 5.5.5
- Windows PowerShell
- ESP32-S3：Armour、Controller
- ESP32-C3：Master、Motor

总仓库根目录不是单独的 ESP-IDF 工程，编译时必须进入对应工程目录。四个工程通过 `../components` 使用共享组件。

## 编译

先打开 ESP-IDF 环境，然后分别编译需要烧录的工程：

```powershell
cd C:\Project\ESP32\Works\RLogo\RLogo_Armour
idf.py build

cd ..\RLogo_Master
idf.py build

cd ..\RLogo_Controller
idf.py build

cd ..\RLogo_Motor
idf.py build
```

也可以先确认目标芯片：

```powershell
idf.py set-target esp32s3   # Armour 或 Controller
idf.py set-target esp32c3   # Master 或 Motor
```

烧录示例：

```powershell
idf.py -p COMx flash monitor
```

其中 `COMx` 替换为实际串口。建议先烧录 Master、Motor 和 Armour，再烧录 Controller。当前仓库只忽略 ESP-IDF 的 `build` 目录和本地 `sdkconfig.old`，生成的二进制不会进入 Git。

## Armour 配置

每块 Armour 烧录前需要设置自己的 ID：

```powershell
cd C:\Project\ESP32\Works\RLogo\RLogo_Armour
idf.py menuconfig
```

进入 `PowerRune Project Configuration`，设置 `Armour Board ID (1-5)`。也可以直接修改当前工程的 `sdkconfig`：

```text
CONFIG_ARMOUR_ID=1
```

压力传感器 UART 默认参数：

- UART1
- 波特率 `460800`
- RX：GPIO5
- TX：GPIO4

Armour 的第一次击中阈值定义在：

```text
components/pr_common/include/pr_types.h
```

当前默认值为 `PR_SENSOR_HIT_THRESHOLD=20`。修改后需要重新编译并烧录 Armour。

## Controller 串口

Controller 使用板载 USB-JTAG 虚拟串口，命令以换行结尾，命令和参数不区分大小写。启动后输入 `HELP` 可以查看帮助。

### 基本命令

```text
HELP
STATUS
CONFIG color=red mode=big loop=off dir=cw
START
RUN color=blue mode=small loop=on dir=ccw
STOP
UNLOCK
OTA
```

`CONFIG` 修改并发送当前配置，`START` 使用最近一次配置开始，`RUN` 可以一次性修改配置并开始。

配置参数：

- `color=red|blue`
- `mode=big|small`
- `loop=on|off`
- `dir=cw|ccw|cs`

### `dir=cs` 静止电机模式

`dir=cs` 表示继续运行 Armour、击中检测和 Master 游戏流程，但 Master 不会自动解锁或启动 Motor。单独发送 `UNLOCK` 也会被拒绝，避免电机运动。

```text
CONFIG color=red mode=big loop=off dir=cs
START
```

`STOP` 仍然有效，可以随时停止当前流程。

### 红灯弹道调试模式

```text
REDLED ON
REDLED OFF
```

- `REDLED ON` 只能在游戏停止时执行，Master 灯带持续红色。
- 此模式不会启动游戏、Armour 或 Motor。
- 模式开启后，`START`、`RUN`、`UNLOCK` 会被 Master 拒绝。
- `REDLED OFF` 退出模式并熄灭 Master 灯带；下一次开始游戏时会按配置恢复灯效。

推荐流程：

```text
STOP
REDLED ON
# 调整弹道
REDLED OFF
```

### 调试输出

```text
DEBUG ON
DEBUG OFF
```

默认只输出连接状态、warning、error 和关键击中信息。`DEBUG ON` 后会显示更详细的协议和设备状态日志。

### Master 第二次 peak 阈值

Armour 会先完成本地击中判断，并把候选击中和 `peak` 上报给 Master。Master 可以对每块 Armour 单独进行第二次 peak 判定：

```text
THRESHOLD SET 1 35.00
THRESHOLD SHOW
THRESHOLD SHOW 1
THRESHOLD OFF 1
```

- `THRESHOLD SET <armour 1-5> <peak 0.00-255.00>`：启用并保存指定 Armour 的第二次阈值。
- `THRESHOLD OFF <armour 1-5>`：关闭指定 Armour 的第二次阈值并保存。
- `THRESHOLD SHOW [armour 1-5]`：查看全部或指定 Armour 的配置。
- 阈值保存在 Master 的 NVS 中，断电后保留。
- 默认不启用第二次阈值判定。
- 阈值配置应在游戏停止时修改。

### 典型使用流程

正常游戏：

```text
STATUS
CONFIG color=red mode=big loop=off dir=cw
START
# 运行过程中观察 HIT armour=... ring=... peak=...
STOP
```

电机静止、只测试 Armour 和击中流程：

```text
CONFIG color=red mode=big loop=off dir=cs
START
STOP
```

启用指定 Armour 的第二次判定后开始游戏：

```text
STOP
THRESHOLD SET 1 35.00
THRESHOLD SET 2 40.00
CONFIG color=red mode=big loop=off dir=cw
START
```

## 状态和故障提示

Controller 可能看到以下类型的信息：

- `TCP connected to Master`：Controller 与 Master 已连接。
- `TCP not connected to Master; retrying`：Controller 正在重连 Master。
- `ARMOUR WARNING online=... missing=...`：当前缺少部分 Armour。
- `READY motor online; armours=...`：Motor 已上线，系统按当前在线 Armour 运行。
- `HIT armour=... ring=... peak=...`：Master 判定有效击中。
- `HIT_ACK_TIMEOUT`：Master 在 100 ms 内未收到 Armour 的击中判定回执，按拒绝处理并输出 warning。

## 目录约定

- 修改通信数据结构或事件号：优先修改 `components/pr_common`。
- 修改 Controller 串口命令：修改 `RLogo_Controller/main/controller.cpp`。
- 修改 Master 命令网关：修改 `RLogo_Master/main/controller_gateway.cpp`。
- 修改 Master 游戏和电机流程：修改 `RLogo_Master/main/mech_engine.cpp`。
- 修改 Armour 本地击中检测：修改 `RLogo_Armour/main/sensor_processor.cpp` 或 `hit_filter.cpp`。
- 修改电机控制：修改 `RLogo_Motor/main`。

修改共享协议后，至少重新编译 Controller 和 Master；如果修改了 Armour 的传感器或上报逻辑，则还需要重新编译并烧录 Armour。

