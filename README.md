# Python 3D Flight Simulator

这是一个使用 Python 和 Panda3D 引擎开发的 3D 飞行模拟游戏项目。它集成了 PyBullet 物理引擎来实现真实的空气动力学和飞行物理模拟。

## 技术栈

*   **游戏引擎**: [Panda3D](https://www.panda3d.org/)
*   **物理引擎**: [PyBullet](https://pybullet.org/wordpress/)
*   **语言**: Python 3.x

## 核心特性

*   **完整的 3D 渲染管线**: 基于 Panda3D，支持光照、材质和场景管理。
*   **真实飞行物理**: 集成 PyBullet，实现升力、阻力、推力、重力及惯性运动。
*   **多视角切换**: 支持第一人称和第三人称视角切换。
*   **模块化设计**: 物理、控制和渲染逻辑分离。

## 快速开始

### 1. 安装依赖

确保您已安装 Python 3.x。

```bash
pip install -r requirements.txt
```

### 2. 运行游戏

```bash
python src/main.py
```

## 飞行控制

| 按键 | 功能 |
| :--- | :--- |
| **W** | 俯仰 (Pitch) - 机头向下 (Nose Down) |
| **S** | 俯仰 (Pitch) - 机头向上 (Nose Up) |
| **A** | 滚转 (Roll) - 左滚 (Left Roll) |
| **D** | 滚转 (Roll) - 右滚 (Right Roll) |
| **Q** | 偏航 (Yaw) - 左偏 (Left Yaw) |
| **E** | 偏航 (Yaw) - 右偏 (Right Yaw) |
| **Shift** | 增加油门 (Throttle Up) |
| **Ctrl** | 减少油门 (Throttle Down) |
| **V** | 切换视角 (第一/第三人称) |
| **Esc** | 退出游戏 |

## 项目结构

```
flight_sim_game/
├── src/
│   ├── main.py                 # 游戏主入口和Panda3D应用
│   ├── physics_manager.py      # PyBullet集成和自定义飞行动力学计算
│   └── aircraft_controller.py  # 用户输入处理和视角控制
├── models/                     # 存放3D模型资源 (Placeholder)
├── requirements.txt            # Python依赖列表
└── README.md                   # 项目说明
```

## 文档与参考

*   **开发指南与3D建模规范**: [DEVELOPMENT.md](DEVELOPMENT.md)
*   **API 参考**: [API_REFERENCE.md](API_REFERENCE.md)
*   **贡献规范**: [CONTRIBUTING.md](CONTRIBUTING.md)

