# Pendulum Ride Physical Simulation

大摆锤游乐设施物理仿真系统 - 模拟大型摆锤游乐设备的完整动力学行为，包括电机驱动加速、重力摆动和制动减速三个阶段。

## Features

- 三阶段运动仿真：加速期、稳定期、制动期
- 支持 360° 过顶摆动模拟
- 游客体验分析：加速度、支撑力、冲击感
- 自转座舱效应：科里奥利力计算
- 安全约束验证：最大加速度、速度限制

## Project Structure

```
Pendulum/
├── component/                 # 核心物理仿真组件
│   ├── GravityMotion.py       # 重力驱动运动计算
│   ├── ForceSinglePeriod.py   # 电机力矩驱动仿真
│   ├── break_down.py          # 制动控制与减速仿真
│   └── SimpleRelease.py       # 简单摆释放模型
├── src/
│   └── TouristState.py        # 游客状态数据结构
├── utils/
│   ├── tools.py               # 转换、计算、数据处理工具
│   └── plot_status.py         # 可视化绘图工具
├── doc/                       # 文档与结果图表
│   └── pendulum.md            # 详细物理模型文档
├── pendulum_overTop.py        # 主仿真脚本 (360°过顶)
├── selfRotateJsonProcess.py   # 自转效应后处理
├── plot_data.ipynb            # 数据分析 Jupyter 笔记本
└── state_s120.json            # 仿真输出数据
```

## Quick Start

### Dependencies

```bash
pip install numpy scipy matplotlib
```

### Run Simulation

```bash
# 运行主仿真程序
python pendulum_overTop.py

# 添加自转效应后处理
python selfRotateJsonProcess.py
```

### Visualization

```python
from utils.plot_status import plot_dict
import json

with open('state_s120.json', 'r') as f:
    data = json.load(f)
plot_dict(data)
```

## Physical Model

### Three-Phase Motion

| Phase | Duration | Description |
|-------|----------|-------------|
| 加速期 | 0-40s | 电机施加恒定力矩，逐步增大摆动幅度 |
| 稳定期 | 40-80s | 纯重力作用下的 360° 过顶摆动 |
| 制动期 | 80-120s | 反向力矩控制减速至停止 |

### Core Equations

**重力驱动运动：**
$$
\omega^2(\theta) = \omega_0^2 + \frac{2g}{L}[\cos(\theta) - \cos(\theta_0)]
$$

**电机力矩驱动：**
$$
\omega^2(\theta) = \omega_0^2 + \frac{2\tau}{I}(\theta-\theta_0) + \frac{2g}{L}[\cos(\theta) - \cos(\theta_0)]
$$

**游客感受加速度（含自转）：**
- 科里奥利加速度: $a_c = 2\omega_s v_p$
- 支撑力分解: $a_{support} = a_{centripetal} + a_{tangential} + g\cos(\theta)$

## Simulation Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| L | 18.0 | m | 摆臂长度 |
| m | 5000 | kg | 系统质量 |
| g | 9.81 | m/s² | 重力加速度 |
| θ_target | 180° | - | 目标最大角度 |
| ω_s | 0.4 | rad/s | 座舱自转角速度 |
| T_total | 120 | s | 总运行时间 |

## Safety Constraints

- 最大角加速度: ≤ 3·g/L
- 最大线速度: ≤ 27.78 m/s (100 km/h)
- 加速期最少摆动次数: ≥ 8

## Results Summary

| Metric | Value |
|--------|-------|
| 最大刺激值 | 4.488 |
| 平均刺激值 | 1.406 |
| 最大总加速度 | 85.71 m/s² (≈8.74g) |
| 平均加速度 | 33.23 m/s² |
| 最大冲击 (jerk) | 24.91 m/s³ |

## Symbol Reference

| Symbol | Meaning | Unit | Type |
|--------|---------|------|------|
| θ | 角位置 | rad / ° | 状态变量 |
| ω | 角速度 | rad/s | 状态变量 |
| L | 摆臂长度 | m | 参数 |
| M | 系统质量 | kg | 参数 |
| g | 重力加速度 | m/s² | 常数 |
| v | 线速度 | m/s | 导出量 |
| τ | 力矩 | N·m | 控制输入 |
| a_c | 向心加速度 | m/s² | 输出 |
| a_t | 切向加速度 | m/s² | 输出 |
| ω_s | 自转角速度 | rad/s | 参数 |

## Documentation

详细的物理模型推导和分析请参阅 [doc/pendulum.md](doc/pendulum.md)

## Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 2.0 | 2025-11-23 | 完善项目文档，添加三阶段仿真说明 | Phoenizard |
| 1.0 | 2025-10-22 | Initial specification | Phoenizard |

## License

MIT License
