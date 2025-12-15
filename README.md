# BasicCar - 基于STM32H7的智能寻线小车

## 项目概述

BasicCar 是一个基于 STM32H750VBT6 微控制器的智能寻线小车控制系统。该项目实现了完整的寻线控制功能，包括 PID 控制算法、IMU姿态解算、参数存储、串口通信、OLED显示和按键交互等功能。

## 主要特性

- ✅ **高性能控制器**: 采用 STM32H7 系列 (480MHz Cortex-M7)
- ✅ **智能寻线**: 基于灰度传感器的寻线算法 + IMU航向保持
- ✅ **双PID控制**: 转向PID + 航向保持PID (基于IMU)
- ✅ **IMU姿态解算**: 集成 ICM45686 九轴传感器，实时输出 Yaw/Pitch/Roll
- ✅ **OLED显示**: 128x64 OLED 显示屏，实时显示姿态角和UI界面
- ✅ **参数持久化**: 使用 W25Q64 Flash 芯片存储 PID 参数
- ✅ **串口调参**: 支持通过串口实时修改和保存 PID 参数
- ✅ **多按键UI**: 三按键交互界面 (上/下/确认) + 问题选择功能
- ✅ **实时调试**: 集成 SEGGER RTT 调试输出

## 硬件配置

### 主控制器
- **MCU**: STM32H750VBT6
  - CPU: ARM Cortex-M7 @ 480MHz
  - Flash: 128KB
  - SRAM: 1MB
  - 支持 D-Cache 和 I-Cache

### 外设接口

| 外设 | 接口 | 用途 | 引脚 |
|------|------|------|------|
| 灰度传感器 | GPIO | 4路寻线传感器 | PA15, PA12, PA11, PA10, PA8 |
| 电机驱动 | TIM1 PWM | 双电机控制 | - |
| Flash 存储 | SPI2 | W25Q64 参数存储 | SPI2_CS |
| IMU传感器 | SPI6 | ICM45686 九轴传感器 | SPI6 |
| OLED显示屏 | I2C4 | SSD1306 (128x64) | I2C4 (0x78) |
| 串口通信 | USART3 | 参数配置 | - |
| 调试输出 | RTT | SEGGER RTT | - |
| 用户按键 | GPIO | 主按键 (单击/长按) | PC13 (USER_KEY) |
| UI按键 | GPIO | 三键UI (上/下/确认) | PE6 (Button1), PE4 (Button2), PE5 (Button3) |

### 电机驱动
- **驱动芯片**: DRV8870 (慢衰减模式)
- **PWM 频率**: 20kHz
- **控制方式**: 双 PWM 控制 (方向 + 速度)

## 软件架构

### 目录结构

```
BasicCar/
├── Core/                      # 核心代码
│   ├── Inc/                   # 头文件
│   │   ├── main.h
│   │   ├── app_entry.h        # C++ 应用入口
│   │   └── ...
│   └── Src/                   # 源文件
│       ├── main.c             # 主程序
│       ├── app_entry.cpp      # C++ 应用实现
│       └── ...
├── Drivers/
│   ├── BSP/                   # 板级支持包
│   │   ├── Inc/
│   │   │   ├── LineFollower.h          # 寻线控制类
│   │   │   ├── LineFollower_Interface.h # 寻线C接口
│   │   │   ├── Pid.hpp                 # PID 控制器模板
│   │   │   ├── PidStorage.hpp          # PID 参数存储
│   │   │   ├── W25Q64.hpp              # Flash 驱动
│   │   │   ├── button.hpp              # 按键驱动
│   │   │   ├── UartRingBuffer.hpp      # 串口环形缓冲区
│   │   │   ├── App_PidConfig.h         # PID 配置管理
│   │   │   └── ui.h                    # UI界面头文件
│   │   ├── Src/
│   │   │   ├── LineFollower_Interface.cpp
│   │   │   ├── App_PidConfig.cpp
│   │   │   ├── button.cpp
│   │   │   └── ui.cpp                  # UI界面实现
│   │   ├── ICM45686/              # IMU驱动
│   │   │   ├── IMU.h/.c           # IMU接口
│   │   │   └── inv_imu_*.h/.c     # ICM45686底层驱动
│   │   ├── U8G2/                  # OLED显示库
│   │   │   ├── OLED.h/.c          # OLED封装
│   │   │   ├── u8g2.h             # U8G2图形库
│   │   │   └── u8g2_*.c           # U8G2实现
│   │   └── SEGGER-RTT/            # RTT调试
│   └── CMSIS/                     # CMSIS 库
│   └── STM32H7xx_HAL_Driver/      # HAL 驱动库
├── CMakeLists.txt                 # CMake 构建配置
└── BasicCar.ioc                   # STM32CubeMX 配置文件
```

### 代码架构

项目采用 **C/C++ 混合编程**架构：

```
+------------------+
|    main.c (C)    |  ← HAL 初始化
+------------------+
         ↓
+------------------+
| app_entry.cpp    |  ← C++ 应用入口
|    (C++)         |
+------------------+
         ↓
+----------------------------------+
| LineFollower     |  ← 寻线控制   |
| PID Controller   |  ← 双PID算法  |
| IMU Driver       |  ← 姿态解算   |
| UI System        |  ← OLED界面   |
| Flash Storage    |  ← 参数存储   |
| UART Parser      |  ← 串口通信   |
+----------------------------------+
```

## 核心功能模块

### 1. 寻线控制 (LineFollower)

**文件**: `Drivers/BSP/Inc/LineFollower.h`

**功能**:
- 读取灰度传感器数据 (实际使用 4-5 路)
- 计算寻线位置偏差
- 使用双 PID 控制器调节电机速度
  - **转向 PID (LPID)**: 根据偏差调整左右轮差速
  - **航向保持 PID (FPID)**: 基于 IMU Yaw 角保持直线行驶

**核心算法**:
```cpp
// 1. 读取传感器并计算位置偏差
float position_error = calculatePositionError();

// 2. 转向PID计算调整量
float turn_adjust = _pidTurn.compute(0.0f, position_error);

// 3. 航向保持PID (基于IMU)
float yaw_now = User_YPR[0];  // 读取当前Yaw角
float yaw_err = wrapAngleDeg(_yaw_ref_deg - yaw_now);
float yaw_adjust = _pidForward.compute(0.0f, yaw_err);

// 4. 混合速度控制
float diff = -yaw_adjust;  // 航向修正量
float speed_l = _base_speed - diff;
float speed_r = _base_speed + diff;
```

**传感器映射**:
```
PA15    PA12  PA11  PA10  PA8
 -5      2     0     2     5
```
- 实际使用 5 路传感器
- 使用加权平均计算位置偏差
- IMU Yaw 角用于直线段航向保持

### 2. PID 控制器 (PidController)

**文件**: `Drivers/BSP/Inc/Pid.hpp`

**特性**:
- 泛型模板实现，支持 float/double
- 经典 PID 算法：P + I + D
- 输出限幅 (防止溢出)
- 积分抗饱和

**公式**:
```
output = Kp * error + Ki * ∫error + Kd * (error - last_error)
```

**参数说明**:
- `Kp`: 比例系数 - 决定响应速度
- `Ki`: 积分系数 - 消除稳态误差
- `Kd`: 微分系数 - 抑制超调

### 3. Flash 参数存储 (PidStorage + W25Q64)

**文件**: 
- `Drivers/BSP/Inc/W25Q64.hpp` - Flash 驱动
- `Drivers/BSP/Inc/PidStorage.hpp` - 参数管理

**存储格式**:
```cpp
struct FlashLayout {
    uint32_t magic;           // 魔数校验 (0x5AA5)
    PidConfig pids[4];        // 最多 4 组 PID 参数
};

struct PidConfig {
    float kp;
    float ki;
    float kd;
};
```

**存储地址**: `0x7FF000` (W25Q64 最后一个 4KB 扇区)

**操作流程**:
1. **上电加载**: 读取 Flash → 校验魔数 → 加载参数
2. **在线修改**: 更新 RAM 缓存 → 立即生效
3. **保存参数**: 擦除扇区 → 写入数据 → 回读校验

### 4. 串口命令解析 (UartRingBuffer)

**文件**: `Drivers/BSP/Inc/UartRingBuffer.hpp`

**特性**:
- 基于 DMA 的环形缓冲区
- 非阻塞解析
- 支持 H7 D-Cache 同步

**命令格式**:
```
&<PID名称>.P=<kp>,I=<ki>,D=<kd>#
```

**示例命令**:
```
&LPID.P=1.5,I=0.2,D=0.5#  // 设置转向 PID
&FPID.P=1.0,I=0.0,D=0.0#  // 设置前进 PID
SAVE                       // 保存参数到 Flash
```

**PID 名称映射**:
- `LPID` → ID 0 (转向 PID)
- `FPID` → ID 1 (前进 PID)

### 5. 按键驱动 (Button)

**文件**: `Drivers/BSP/Inc/button.hpp`

**特性**:
- 状态机实现
- 软件消抖 (20ms)
- 支持单击和长按 (1000ms)
- 回调函数机制

**使用示例**:
```cpp
Button btn(USER_KEY_GPIO_Port, USER_KEY_Pin, true);

void onOkClick() {
    // 按键单击处理
    App_Pid_Save();
}

void onLongPress() {
    // 长按重置航向
    LineFollower_SetYaw();
}

btn.attachClick(onOkClick);
btn.attachLongPress(onLongPress);
```

### 6. IMU姿态解算 (ICM45686)

**文件**: `Drivers/BSP/ICM45686/IMU.h`

**功能**:
- 九轴传感器 (加速度计 + 陀螺仪 + 磁力计)
- AHRS姿态解算，输出四元数和欧拉角
- 实时输出 Yaw/Pitch/Roll 角度

**接口**:
```c
void IMU_init(void);                          // 初始化IMU
void IMU_getYawPitchRoll(float * ypr);        // 获取欧拉角
void IMU_AHRSupdate(...);                     // AHRS解算更新
```

**应用**:
- 提供实时 Yaw 角给航向保持PID
- 在 OLED 上显示姿态角度
- 用于直线段的航向保持和修正

### 7. UI界面系统 (U8G2 + OLED)

**文件**: 
- `Drivers/BSP/Inc/ui.h`
- `Drivers/BSP/Src/ui.cpp`
- `Drivers/BSP/U8G2/` (图形库)

**功能**:
- 128x64 OLED显示屏 (SSD1306, I2C接口)
- 实时显示 Yaw/Pitch/Roll 姿态角
- 问题选择界面 (Q1-Q4)
- 三按键交互 (上/下/确认)

**UI布局**:
```
+---------------------------+
| BasicCar UI          Q1   |  ← 顶部标题栏
+---------------------------+
| [Q1] Q2  Q3  Q4           |  ← 问题选择器
+---------------------------+
| Yaw:  123.45              |  ← 姿态角显示
| Pit:   45.67              |
| Rol:  -12.34              |
+---------------------------+
```

**接口**:
```c
void UI_Init(void);              // 初始化UI
void UI_Button_Update(void);     // 更新按键状态
void UI_Render(void);            // 渲染UI到OLED
uint8_t UI_GetConfirmedQuestion(void);  // 获取确认的题号
```

## 程序流程

### 主程序流程 (main.c)

```
1. MPU 配置
2. 使能 I-Cache 和 D-Cache
3. HAL 初始化
4. 系统时钟配置 (480MHz)
5. 外设初始化
   ├── GPIO
   ├── DMA
   ├── TIM1/2/3/4/7 (定时器)
   ├── FDCAN2
   ├── SPI2/6 (Flash & IMU)
   ├── I2C4 (OLED)
   └── USART2/3/4/7
6. SEGGER RTT 初始化
7. 寻线控制初始化
8. PID 参数加载 (从Flash)
9. 串口 DMA 启动
10. IMU 初始化
11. 启动定时器中断 (TIM7, 1kHz)
12. OLED 初始化 (U8G2)
13. 进入 C++ 主循环 (App_Start)
```

### C++ 应用流程 (app_entry.cpp)

```
App_Start() {
    // 1. 初始化按键
    btn.attachClick(onOkClick);        // 单击: 保存PID参数
    btn.attachLongPress(setYawRef);    // 长按: 重置航向参考
    
    // 2. 初始化UI
    UI_Init();
    
    // 3. 主循环
    while(1) {
        // 扫描主按键 (PC13)
        btn.scan();
        
        // 扫描UI按键 (PE6/PE4/PE5)
        UI_Button_Update();
        
        // 处理串口命令
        App_Serial_Loop();
        
        // 绘制UI到OLED
        UI_Render();
    }
}
```

### 定时器中断流程 (1ms)

```
TIM7 中断 (1kHz)
    ↓
HAL_TIM_PeriodElapsedCallback()
    ↓
LineFollower_OnTimer()
    ↓
1. 读取灰度传感器
2. 计算位置偏差
3. 读取IMU Yaw角
4. 航向保持PID计算
5. 转向PID计算 (可选)
6. 混合差速控制
7. 更新电机 PWM
```

## 使用说明

### 初次使用

1. **烧录程序**: 使用 STM32CubeProgrammer 或 OpenOCD 烧录固件
2. **连接串口**: USART3，波特率 115200，8N1 格式
3. **观察OLED**: 屏幕应显示UI界面和实时姿态角
4. **放置小车**: 将小车放在黑色寻线轨道上
5. **使用UI**: 通过三个按键 (PE6上/PE4下/PE5确认) 选择问题 Q1-Q4
6. **观察运行**: 小车应自动沿轨道行驶并保持航向

### PID 参数调试

#### 方法1: 串口调参

使用串口工具发送命令：

```bash
# 设置转向 PID (LPID, ID=0)
&LPID.P=1.5,I=0.2,D=0.5#

# 设置航向保持 PID (FPID, ID=1)
&FPID.P=1.0,I=0.0,D=0.0#

# 保存参数到 Flash
SAVE
```

#### 方法2: 代码修改

修改 `PidStorage.hpp` 中的默认值：

```cpp
_cache.pids[PID_ID_TURN] = {0.1f, 0.0f, 0.2f};      // 转向PID
_cache.pids[PID_ID_FORWARD] = {1.0f, 0.0f, 0.0f};  // 航向保持PID
```

#### 方法3: 按键保存

- 单击 PC13 按键: 保存当前PID参数到Flash
- 长按 PC13 按键: 重置航向参考角度

### UI界面操作

**三按键功能**:
- **PE6 (Button1/Up)**: 向上选择问题 (Q1→Q4循环)
- **PE4 (Button2/Down)**: 向下选择问题 (Q4→Q1循环)
- **PE5 (Button3/OK)**: 确认当前选择的问题

**OLED显示内容**:
- 顶部标题栏: 显示程序名和当前选中/确认的问题
- 问题选择器: Q1-Q4四个选项，选中反显，确认显示外框
- 姿态角显示: 实时显示 Yaw/Pitch/Roll 三个角度

### 调试输出

使用 SEGGER RTT Viewer 查看实时日志：

```
[System] PID loaded from Flash.
[System] Applied PID: Turn P=1.500000 I=0.200000 D=0.500000
YawErr: 2.34, YawAdj: 0.12, Diff: -0.12
Button Clicked!
[Ack] Set LPID (ID 0): P=1.500000, I=0.200000, D=0.500000
```

## 编译和调试

### 编译环境

- **工具链**: ARM GCC (arm-none-eabi-gcc) 10.3+ 推荐
- **构建系统**: CMake 3.22+
- **IDE**: CLion / VS Code (推荐)

### 编译步骤

```bash
# 1. 配置 CMake
mkdir build && cd build
cmake ..

# 2. 编译
make -j$(nproc)

# 3. 生成输出文件
# BasicCar.elf - 可执行文件
# BasicCar.bin - 二进制文件
# BasicCar.hex - Intel HEX 文件
```

### 调试配置

#### 使用 OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

#### 使用 SEGGER J-Link

```bash
JLinkGDBServer -device STM32H750VB -if SWD -speed 4000
```

### 常用 GDB 命令

```bash
# 连接目标
target remote localhost:3333

# 加载程序
load

# 运行
continue

# 设置断点
break main
```

## PID 调参建议

### 调参步骤

1. **先调航向保持 PID (FPID)**:
   - 这是基于 IMU Yaw 角的航向保持
   - 从小的 Kp 开始 (如 0.5~1.0)
   - 观察小车在直线段能否保持航向
   - 逐步增大 Kp 直到响应快速但不震荡
   - 添加少量 Kd 抑制震荡
   - Ki 通常保持为 0

2. **可选: 调寻线 PID (LPID)**:
   - 目前代码中主要使用航向保持，寻线PID可作为辅助
   - 参数调整方法类似
   - 注意两个PID的输出会叠加影响

### 参数范围建议

| 参数 | 转向PID (LPID) | 航向保持PID (FPID) |
|------|----------------|-------------------|
| Kp   | 0.1 ~ 2.0      | 0.5 ~ 2.0         |
| Ki   | 0.0 ~ 0.2      | 0.0 ~ 0.1         |
| Kd   | 0.1 ~ 0.5      | 0.0 ~ 0.5         |

**当前默认值**:
- LPID: P=0.1, I=0.0, D=0.2
- FPID: P=1.0, I=0.0, D=0.0

### 常见问题

**问题1**: 小车左右摆动严重
- **原因**: Kp 过大或 Kd 过小
- **解决**: 降低 Kp 或增大 Kd

**问题2**: 小车反应迟钝，容易冲出轨道
- **原因**: 航向保持PID的 Kp 过小
- **解决**: 增大 FPID 的 Kp

**问题3**: 小车在直线上逐渐偏离航向
- **原因**: 存在稳态误差或航向参考未设置
- **解决**: 长按 PC13 重置航向参考，或添加 Ki 项

**问题4**: IMU数据异常
- **原因**: SPI6通信问题或传感器未初始化
- **解决**: 检查OLED上的Yaw/Pitch/Roll值是否合理

## 技术亮点

1. **C/C++ 混合编程**: 
   - C 语言处理 HAL 初始化
   - C++ 实现业务逻辑和类封装
   - extern "C" 实现 C/C++ 互调

2. **模板化设计**:
   - PID 控制器使用泛型模板
   - 支持不同数据类型 (float/double)

3. **高效缓存管理**:
   - STM32H7 的 D-Cache 同步处理
   - DMA 缓冲区的 Cache 失效操作

4. **实时性保证**:
   - 1kHz 定时器中断执行寻线控制
   - 非阻塞串口解析
   - OLED异步刷新

5. **参数持久化**:
   - Flash 存储 PID 参数
   - 掉电不丢失配置

6. **IMU姿态融合**:
   - 九轴传感器 AHRS 解算
   - 实时输出欧拉角
   - 用于航向保持和姿态监控

7. **完整的UI系统**:
   - U8G2 图形库驱动 OLED
   - 多按键交互界面
   - 实时显示状态和参数

## 扩展功能建议

- [x] ~~增加 OLED 显示屏显示实时参数~~ (已实现)
- [x] ~~添加 IMU 姿态传感器~~ (已实现)
- [ ] 添加蓝牙/WiFi 模块实现无线调参
- [ ] 实现多种运动模式 (慢速/快速/精确)
- [ ] 添加障碍物检测功能
- [ ] 实现 PID 自整定算法
- [ ] 完善问题选择功能 (Q1-Q4) 对应的不同运行模式
- [ ] 添加数据记录功能 (轨迹/速度/姿态)

## 许可证

本项目中 STM32 HAL 驱动和 CMSIS 库遵循 STMicroelectronics 许可证。

其他代码遵循项目根目录的 LICENSE 文件。

## 作者

TheWinds071

## 更新日志

### 2025-12-15
- 添加 ICM45686 九轴 IMU 支持
- 实现基于 IMU 的航向保持功能
- 集成 U8G2 OLED 显示库
- 添加完整的 UI 界面系统
- 实现三按键交互 (上/下/确认)
- 添加问题选择功能 (Q1-Q4)
- 更新 README 文档

### 2025-12-11
- 初始版本发布
- 实现基本寻线功能
- 添加 PID 参数存储
- 支持串口调参

---

**如有问题或建议，欢迎提交 Issue！**
