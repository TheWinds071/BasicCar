# BasicCar - 2024年全国大学生电子设计竞赛H题智能小车

## 赛题背景

本项目为 **2024年全国大学生电子设计竞赛 H 题（智能送药小车）** 的参赛作品代码实现。赛题要求设计一个智能小车，能够在矩形赛道上完成精确定位、路径规划和循线行驶等任务。

## 项目概述

BasicCar 是一个基于 STM32H750VBT6 高性能微控制器的智能寻线小车控制系统。该系统完整实现了 2024 电赛 H 题的全部四道题目要求（Q1-Q4），包括：
- **Q1**: A点到B点直线行驶并停车
- **Q2**: 按 A→B→C→D→A 路径完成一圈
- **Q3**: 按 A→C→B→D→A 对角线路径完成一圈
- **Q4**: 按 Q3 路径连续完成四圈

系统集成了高精度 PID 控制算法、九轴 IMU 姿态解算、参数持久化存储、实时 OLED 显示和多按键交互等功能模块。

## 赛题赛道布局

赛题采用矩形赛道，四个角点分别为 A、B、C、D（顺时针排列）：

```
        B ────────── C
        │           │
        │           │  
半圆弧线│           │半圆弧线
        │           │
        │           │
        A ────────── D
      (起点)
```

**赛道特征**：
- A-B、C-D 为直线段（无地面标线）
- B-C、D-A 为半圆弧段（黑色寻线轨迹）
- A、B、C、D 四点有地面标识（黑色标记线）
- 小车起点位于 A 点

## 赛题要求详解

### Q1：直线行驶（基础题）
**任务**: 从 A 点出发，沿直线行驶到 B 点并停车，到达 B 点时蜂鸣器提示。

**技术要点**:
- 使用 IMU 航向保持，沿直线精确行驶
- 通过灰度传感器检测 B 点标识
- 到达目标点后精确停车

### Q2：标准矩形路径（发挥题1）
**任务**: 从 A 点出发，按 A→B→C→D→A 路径行驶一圈，每到达一个点蜂鸣器提示一次。

**技术要点**:
- 直线段（A→B、C→D）使用航向保持PID
- 半圆段（B→C、D→A）使用寻线PID跟随轨迹
- 状态机自动切换各段行驶模式
- 精确的边沿检测识别各点位

### Q3：对角线路径（发挥题2）
**任务**: 从 A 点出发，按 A→C→B→D→A 对角线路径行驶一圈，每到达一个点蜂鸣器提示一次。

**技术要点**:
- 需要跨越赛道对角线（A→C、B→D）
- 航向角度精确控制（转向约178°）
- 混合使用直线航向保持和弧线循线
- 更复杂的状态机逻辑

### Q4：多圈连续行驶（发挥题3）
**任务**: 按 Q3 路径（A→C→B→D→A）连续行驶 4 圈，完成后停车。

**技术要点**:
- 圈数计数与状态保持
- 长时间稳定运行能力
- 累积误差控制
- 多圈后精确停车

## 主要特性

- ✅ **完整实现电赛H题**: 支持全部四道题目（Q1-Q4）的自动运行
- ✅ **高性能控制器**: 采用 STM32H7 系列 (480MHz Cortex-M7)
- ✅ **混合控制策略**: 直线段航向保持 + 弧线段循线跟随
- ✅ **双PID控制**: 转向PID (寻线) + 航向保持PID (基于IMU)
- ✅ **IMU姿态解算**: 集成 ICM45686 九轴传感器，实时输出 Yaw/Pitch/Roll
- ✅ **状态机管理**: 每道题目独立状态机，自动切换运行模式
- ✅ **OLED显示**: 128x64 OLED 显示屏，实时显示姿态角和题目选择
- ✅ **参数持久化**: 使用 W25Q64 Flash 芯片存储 PID 参数
- ✅ **串口调参**: 支持通过串口实时修改和保存 PID 参数
- ✅ **多按键UI**: 三按键交互界面 (上/下/确认) + 题目选择功能 (Q1-Q4)
- ✅ **声光提示**: 蜂鸣器在到达各点时自动提示
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
| 灰度传感器 | GPIO | 5路寻线传感器（检测弧线轨迹） | PA15, PA12, PA11, PA10, PA8 |
| 电机驱动 | TIM1 PWM | 双电机差速控制 | - |
| Flash 存储 | SPI2 | W25Q64 PID参数存储 | SPI2_CS |
| IMU传感器 | SPI6 | ICM45686 九轴传感器（航向保持） | SPI6 |
| OLED显示屏 | I2C4 | SSD1306 (128x64) 状态显示 | I2C4 (0x78) |
| 串口通信 | USART3 | PID参数配置 | - |
| 调试输出 | RTT | SEGGER RTT 实时调试 | - |
| 用户按键 | GPIO | 主按键 (单击/长按) | PC13 (USER_KEY) |
| UI按键 | GPIO | 三键UI (上/下/确认) | PE6 (Button1), PE4 (Button2), PE5 (Button3) |
| 蜂鸣器 | GPIO | 到点声音提示 | - |

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
- 实现 Q1-Q4 四道赛题的完整控制逻辑
- 读取灰度传感器数据 (5路传感器)
- 使用状态机管理各题目的运行流程
- 双 PID 控制器调节电机速度
  - **转向 PID (LPID)**: 弧线段根据偏差调整左右轮差速
  - **航向保持 PID (FPID)**: 直线段基于 IMU Yaw 角保持航向

**控制策略**:
```cpp
// 直线段（A→B, C→D）：航向保持
float yaw_now = User_YPR[0];  // 读取当前Yaw角
float yaw_err = wrapAngleDeg(_yaw_ref_deg - yaw_now);
float yaw_adjust = _pidForward.compute(0.0f, yaw_err);
setEndSpeed(0.0f, yaw_adjust);  // 纯航向修正

// 弧线段（B→C, D→A）：循线跟随
float position_error = calculatePositionError();  // 传感器计算偏差
float turn_adjust = _pidTurn.compute(0.0f, position_error);
setEndSpeed(turn_adjust, 0.0f);  // 纯循线转向
```

**传感器映射**:
```
PA15    PA12  PA11  PA10  PA8
 -4      -2     0     2     4  (权重)
```
- 5 路灰度传感器检测黑色弧线
- 加权平均计算位置偏差
- 支持短暂丢线保持方向（弧线段稳定性优化）

**状态机实现**:

每道题目使用独立状态机管理：

- **Q1 状态**: Idle → GoStraight_AB → StopAtB
- **Q2 状态**: Idle → Straight_AB → Arc_BC → Straight_CD → Arc_DA → Done
- **Q3 状态**: Idle → Straight_AC → Arc_CB → Straight_BD → Arc_DA → Done  
- **Q4 状态**: 循环 Q3 状态 4 圈，维护圈数计数器

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
- 题目选择界面 (Q1-Q4) - **电赛四道题目选择**
- 三按键交互 (上/下/确认)
- 状态指示（显示当前选中和确认的题号）

**UI布局**:
```
+---------------------------+
| BasicCar UI          Q1   |  ← 顶部标题栏 (右侧显示当前选中/确认的题号)
+---------------------------+
| [Q1] Q2  Q3  Q4           |  ← 题目选择器 (选中=反显框, 确认=外框)
+---------------------------+
| Yaw:  123.45              |  ← 姿态角实时显示
| Pit:   45.67              |
| Rol:  -12.34              |
+---------------------------+
```

**显示说明**:
- 右上角显示: 
  - "Q1" = 选中Q1但未确认
  - "Q1 OK" = 已确认Q1且正在运行
  - "Q1>Q2" = 当前选中Q1但之前确认了Q2
- 题目选择器: 反显框表示当前选中，外框表示已确认运行
- 姿态角: 实时更新的 Yaw/Pitch/Roll 数值（单位：度）

**接口**:
```c
void UI_Init(void);                        // 初始化UI
void UI_Button_Update(void);               // 更新按键状态
void UI_Render(void);                      // 渲染UI到OLED
uint8_t UI_GetConfirmedQuestion(void);     // 获取确认的题号 (1-4)
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
根据UI确认的题号 (Q1-Q4) 执行对应状态机:
    ├─ Q1: Idle → GoStraight_AB → StopAtB
    ├─ Q2: Idle → Straight_AB → Arc_BC → Straight_CD → Arc_DA → Done
    ├─ Q3: Idle → Straight_AC → Arc_CB → Straight_BD → Arc_DA → Done
    └─ Q4: 循环 Q3 路径 4 圈 → Done
    ↓
每个状态执行相应控制策略:
    ├─ 直线段: driveStraightYawHold() - 航向保持PID
    └─ 弧线段: driveArcLineFollow()   - 循线PID
    ↓
检测状态转换条件:
    ├─ rising edge (无线→有线): 到达 B 或 D 点
    └─ falling edge (有线→无线): 到达 C 或 A 点
    ↓
到达关键点时:
    ├─ 切换状态机状态
    ├─ 重置/更新PID控制器
    ├─ 蜂鸣器提示 (Prompt::once())
    └─ 更新电机PWM输出
```

## 使用说明（电赛操作流程）

### 初次使用

1. **烧录程序**: 使用 STM32CubeProgrammer 或 OpenOCD 烧录固件
2. **连接串口**: USART3，波特率 115200，8N1 格式（可选，用于调试）
3. **观察OLED**: 屏幕应显示UI界面和实时姿态角
4. **放置小车**: 将小车放在 A 点（起始位置）
5. **选择题目**: 
   - 使用 PE6 (上键) 和 PE4 (下键) 选择题目 Q1-Q4
   - 按 PE5 (确认键) 确认选择
6. **开始运行**: 确认后小车自动开始执行对应题目
7. **观察运行**: 
   - Q1: 小车直线行驶到 B 点停车
   - Q2: 小车按 A→B→C→D→A 完成一圈
   - Q3: 小车按 A→C→B→D→A 完成一圈
   - Q4: 小车按 Q3 路径完成 4 圈
8. **声音提示**: 到达每个关键点时蜂鸣器会响一声

### 题目切换操作

**运行中切换题目**:
1. 停止小车（可选：重启系统）
2. 将小车放回 A 点
3. 使用按键重新选择题目
4. 按确认键开始新题目

**注意事项**:
- 每次切换题目后，状态机会自动重置
- 建议在 A 点重新开始，确保初始条件一致
- IMU 航向参考会在新题目开始时自动设置

### PID 参数调试

### 调参方法汇总

#### 方法1: 串口调参（推荐用于现场快速调试）

使用串口工具发送命令：

```bash
# 设置航向保持 PID (FPID, ID=1) - 用于直线段
&FPID.P=1.0,I=0.0,D=0.0#

# 设置循线 PID (LPID, ID=0) - 用于弧线段
&LPID.P=0.1,I=0.0,D=0.2#

# 保存参数到 Flash
SAVE
```

#### 方法2: 代码修改（用于预设默认值）

修改 `PidStorage.hpp` 中的默认值：

```cpp
_cache.pids[PID_ID_TURN] = {0.1f, 0.0f, 0.2f};      // 循线PID (弧线段)
_cache.pids[PID_ID_FORWARD] = {1.0f, 0.0f, 0.0f};  // 航向保持PID (直线段)
```

#### 方法3: 按键保存

- 单击 PC13 按键: 保存当前PID参数到Flash
- 长按 PC13 按键: 重置航向参考角度

### 速度调整

代码中针对不同段设置了不同基速（在 `LineFollower::updateISR()` 中）：

```cpp
// 直线段 (Q1, Q2-AB, Q2-CD, Q3-AC, Q3-BD, Q4-AC, Q4-BD)
setBaseSpeed(0.30f);  // 30% 最大速度

// 弧线段 (Q2-BC, Q2-DA, Q3-CB, Q3-DA, Q4-CB, Q4-DA)  
setBaseSpeed(0.20f);  // 20% 最大速度（弧线需要更慢以保证跟线）
```

根据实际场地情况调整这些速度值。

### UI界面操作

**三按键功能**:
- **PE6 (Button1/Up)**: 向上选择题目 (Q1→Q4循环)
- **PE4 (Button2/Down)**: 向下选择题目 (Q4→Q1循环)
- **PE5 (Button3/OK)**: 确认当前选择的题目并开始运行

**OLED显示内容**:
- 顶部标题栏: 显示程序名和当前选中/确认的题目
- 题目选择器: Q1-Q4 四个选项，选中反显，确认显示外框
- 姿态角显示: 实时显示 Yaw/Pitch/Roll 三个角度（用于调试航向）

**主按键功能** (PC13):
- 单击: 保存当前 PID 参数到 Flash
- 长按: 重置 IMU 航向参考角度（用于校准）

### 调试输出

使用 SEGGER RTT Viewer 查看实时日志：

```
[System] PID loaded from Flash.
[System] Applied PID: Turn P=0.100000 I=0.000000 D=0.200000
[Q2] -> Straight_AB raw=0x0000 yaw=0.00
[Q2] -> Arc_BC raw=0x1000 yaw=2.34
[Q2] -> Straight_CD raw=0x0000 yaw=178.56
[Q2] -> Arc_DA raw=0x0800 yaw=-178.23
[Q2] -> Done raw=0x0000 yaw=0.12
YawErr: 2.34, YawAdj: 0.12, Diff: -0.12
Button Clicked!
[Ack] Set FPID (ID 1): P=1.000000, I=0.000000, D=0.000000
```

**日志说明**:
- `[Q2] -> 状态名`: 状态机切换日志，包含当前传感器原始值和航向角
- `YawErr/YawAdj`: 航向误差和修正量，用于调试航向保持
- `raw=0xXXXX`: 灰度传感器原始值（位掩码）
  - `0x0000`: 无线（直线段）
  - `0xXXXX`: 有线（弧线段，不同位表示不同传感器）

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

## PID 参数调试（电赛调参指南）

### 电赛现场调参建议

根据赛题要求，需要分别优化两种控制模式的 PID 参数：

**1. 航向保持 PID (FPID) - 用于直线段**
   - 作用：在 A→B、C→D 等直线段保持航向稳定
   - 优先级：**最高**（直接影响 Q1-Q4 所有题目）
   - 调参重点：快速响应，避免漂移

**2. 循线 PID (LPID) - 用于弧线段**  
   - 作用：在 B→C、D→A 等半圆弧段跟随黑线
   - 优先级：**次要**（仅 Q2-Q4 需要）
   - 调参重点：平滑跟随，避免震荡

### 调参步骤

#### 阶段1: 调试航向保持 PID (FPID)

建议在 **Q1 题目**上测试，因为 Q1 只有直线段：

1. **从保守参数开始**:
   ```bash
   &FPID.P=0.5,I=0.0,D=0.0#
   ```

2. **逐步增大 Kp**:
   - 观察小车在 A→B 直线上的表现
   - Kp 太小：小车缓慢偏离航向
   - Kp 太大：小车左右摆动
   - 目标：找到响应快但不震荡的 Kp（建议 0.8~1.5）

3. **添加微分 Kd** (可选):
   - 如果存在轻微震荡，添加 Kd（建议 0.0~0.3）
   - 例如: `&FPID.P=1.0,I=0.0,D=0.2#`

4. **保存参数**: 
   - 串口发送: `SAVE`
   - 或单击 PC13 按键保存

#### 阶段2: 调试循线 PID (LPID)

建议在 **Q2 题目**上测试：

1. **从保守参数开始**:
   ```bash
   &LPID.P=0.1,I=0.0,D=0.1#
   ```

2. **观察弧线段表现**:
   - 降低基速（代码中弧线段默认 0.20）
   - 观察 B→C 和 D→A 半圆段的跟线效果

3. **调整 Kp**:
   - Kp 太小：小车反应慢，容易冲出弧线
   - Kp 太大：小车在弧线上来回摆动
   - 建议范围：0.1~0.3

4. **保存参数**: `SAVE`

### 调参方法汇总

### 参数范围建议

| 参数 | 循线PID (LPID) | 航向保持PID (FPID) |
|------|----------------|-------------------|
| Kp   | 0.1 ~ 0.3      | 0.8 ~ 1.5         |
| Ki   | 0.0 ~ 0.1      | 0.0 ~ 0.1         |
| Kd   | 0.1 ~ 0.3      | 0.0 ~ 0.3         |
| **作用场景** | 弧线段 (B→C, D→A) | 直线段 (A→B, C→D) |
| **重要程度** | 次要 | **最高** |

**当前推荐默认值**:
- LPID (循线): P=0.1, I=0.0, D=0.2
- FPID (航向): P=1.0, I=0.0, D=0.0

### 常见问题与解决

**问题1**: Q1 题目中小车在直线上左右摆动严重
- **原因**: 航向保持 PID 的 Kp 过大
- **解决**: 降低 FPID 的 Kp (如从 1.5 降到 1.0)
- **验证**: 重新运行 Q1，观察摆动是否减小

**问题2**: Q1 题目中小车缓慢偏离直线，无法到达 B 点
- **原因**: 航向保持 PID 的 Kp 过小
- **解决**: 增大 FPID 的 Kp (如从 0.5 增到 1.0)
- **验证**: 小车应能快速修正偏离

**问题3**: Q2/Q3 题目中小车在弧线段冲出轨道
- **原因**: 
  1. 循线 PID 的 Kp 过小，反应不及时
  2. 弧线段速度过快
- **解决**: 
  1. 增大 LPID 的 Kp
  2. 降低弧线段基速（修改代码中 `setBaseSpeed(0.20f)` 为更小值）

**问题4**: Q2/Q3 题目中小车在弧线段来回摆动
- **原因**: 循线 PID 的 Kp 过大
- **解决**: 降低 LPID 的 Kp，可添加少量 Kd

**问题5**: 小车在直线上逐渐偏离航向，且调大 Kp 无效
- **原因**: 存在稳态误差，IMU 可能存在零飘
- **解决**: 
  1. 长按 PC13 重置航向参考
  2. 尝试添加小量 Ki（如 0.05~0.1）消除稳态误差
  3. 检查 IMU 安装是否牢固

**问题6**: IMU 数据异常（Yaw角不变化或跳变）
- **原因**: SPI6 通信问题或传感器未正确初始化
- **解决**: 
  1. 检查 OLED 上的 Yaw/Pitch/Roll 值是否合理
  2. 重启系统
  3. 检查 IMU 供电和 SPI 连接

**问题7**: Q4 题目无法完成 4 圈（中途停止或跑飞）
- **原因**: 
  1. PID 参数不够稳定
  2. 累积误差导致后续圈数偏离
- **解决**: 
  1. 先确保 Q3 单圈能稳定完成
  2. 优化直线段和弧线段的 PID 参数
  3. 考虑降低整体速度以提高稳定性

## 技术亮点

1. **完整的电赛题目实现**: 
   - 独立状态机管理 Q1-Q4 四道赛题
   - 自动识别关键点位并切换运行模式
   - 支持题目间快速切换

2. **混合控制策略**:
   - 直线段使用 IMU 航向保持，精度高
   - 弧线段使用灰度传感器循线，鲁棒性强
   - 两种模式无缝切换，避免振荡

3. **C/C++ 混合编程**: 
   - C 语言处理 HAL 初始化和中断
   - C++ 实现业务逻辑和类封装
   - extern "C" 实现 C/C++ 互调

4. **模板化设计**:
   - PID 控制器使用泛型模板
   - 支持不同数据类型 (float/double)
   - 代码复用性强

5. **高效缓存管理**:
   - STM32H7 的 D-Cache 同步处理
   - DMA 缓冲区的 Cache 失效操作
   - 确保高速运行下的数据一致性

6. **实时性保证**:
   - 1kHz 定时器中断执行控制算法
   - 非阻塞串口解析
   - OLED 异步刷新
   - 确保控制周期稳定

7. **参数持久化**:
   - Flash 存储 PID 参数
   - 掉电不丢失配置
   - 现场调试后立即保存

8. **IMU姿态融合**:
   - 九轴传感器 AHRS 解算
   - 实时输出欧拉角
   - 用于航向保持和姿态监控
   - 提供比单纯循线更高的控制精度

9. **完整的UI系统**:
   - U8G2 图形库驱动 OLED
   - 多按键交互界面
   - 实时显示状态和参数
   - 方便现场操作和调试

10. **边沿检测优化**:
    - 时间去抖避免误触发
    - 弧线段采用连续丢线判定
    - 提高状态切换可靠性

## 扩展功能建议

### 已实现功能
- [x] OLED 显示屏显示实时参数
- [x] IMU 姿态传感器集成
- [x] Q1-Q4 全部赛题状态机实现
- [x] 蜂鸣器到点提示
- [x] 参数持久化存储

### 潜在改进方向
- [ ] 添加蓝牙/WiFi 模块实现无线调参（便于现场调试）
- [ ] 实现多种运动模式 (慢速/快速/精确)
- [ ] 添加障碍物检测功能（扩展应用）
- [ ] 实现 PID 自整定算法（减少手动调参工作量）
- [ ] 添加数据记录功能 (轨迹/速度/姿态) 用于赛后分析
- [ ] 优化 Q4 多圈运行的累积误差校正
- [ ] 增加更精确的点位识别算法（视觉或激光）
- [ ] 实现速度自适应控制（根据曲率自动调整）

## 许可证

本项目中 STM32 HAL 驱动和 CMSIS 库遵循 STMicroelectronics 许可证。

其他代码遵循项目根目录的 LICENSE 文件。

## 作者

TheWinds071

## 更新日志

### 2026-01 (当前版本)
- 更新 README 文档，详细说明 2024 电赛 H 题背景
- 添加赛道布局图和四道题目详细说明
- 补充电赛现场操作流程和调参指南
- 完善常见问题与解决方案

### 2025-12-15
- 添加 ICM45686 九轴 IMU 支持
- 实现基于 IMU 的航向保持功能（直线段控制核心）
- 集成 U8G2 OLED 显示库
- 添加完整的 UI 界面系统
- 实现三按键交互 (上/下/确认)
- 添加题目选择功能 (Q1-Q4)
- 实现 Q1-Q4 全部题目的状态机逻辑
- 添加蜂鸣器到点提示功能
- 优化边沿检测算法（去抖和稳定性）

### 2025-12-11
- 初始版本发布
- 实现基本寻线功能
- 添加 PID 参数存储
- 支持串口调参
- 双PID控制器架构搭建

---

## 致谢

感谢 2024 年全国大学生电子设计竞赛组委会提供的竞赛平台。

## 参考资料

- [2024年全国大学生电子设计竞赛赛题](https://nuedc.sjtu.edu.cn/)
- [STM32H7 参考手册](https://www.st.com/en/microcontrollers-microprocessors/stm32h7-series.html)
- [U8G2 图形库文档](https://github.com/olikraus/u8g2)
- [ICM-45686 数据手册](https://invensense.tdk.com/)

---

**如有问题或建议，欢迎提交 Issue！**
