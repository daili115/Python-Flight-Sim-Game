# 3D飞行模拟游戏 API 参考

本项目的主要 API 集中在以下三个核心类中。

## 1. `PhysicsManager` (src/physics_manager.py)

负责 PyBullet 物理世界的初始化、飞行动力学计算和刚体状态同步。

| 方法 | 描述 | 参数 | 返回值 |
| :--- | :--- | :--- | :--- |
| `__init__(self, base)` | 初始化 PyBullet 客户端，设置重力，并定义飞行动力学参数。 | `base`: Panda3D `ShowBase` 实例。 | None |
| `load_aircraft(self, model_path, initial_pos, initial_hpr)` | 加载飞机视觉模型和 PyBullet 刚体，设置初始速度。 | `model_path`: 飞机模型路径。<br>`initial_pos`: 初始位置 (`LPoint3`)。<br>`initial_hpr`: 初始朝向 (`LVector3`)。 | 飞机模型的 Panda3D `NodePath`。 |
| `apply_flight_forces(self, dt)` | **核心方法**。计算并应用重力、推力、升力、阻力以及控制力矩。 | `dt`: 帧时间间隔 (秒)。 | None |
| `update_controls(self, inputs)` | 接收来自控制器的用户输入（油门、控制面）。 | `inputs`: 包含 `throttle`, `aileron`, `elevator`, `rudder` 的字典。 | None |
| `update(self, dt)` | 执行物理世界的步进和力应用。 | `dt`: 帧时间间隔 (秒)。 | None |
| `get_aircraft_state(self)` | 获取飞机的当前位置、姿态和线速度。 | None | `(pos, quat, linear_vel)`: `LPoint3`, `LQuaternion`, `LVector3`。 |

## 2. `AircraftController` (src/aircraft_controller.py)

负责处理用户输入、更新控制面状态以及管理多视角相机。

| 方法 | 描述 | 参数 | 返回值 |
| :--- | :--- | :--- | :--- |
| `__init__(self, base, physics_mgr, aircraft_np)` | 初始化控制输入状态和相机参数，并设置输入监听。 | `base`: Panda3D `ShowBase` 实例。<br>`physics_mgr`: `PhysicsManager` 实例。<br>`aircraft_np`: 飞机 `NodePath`。 | None |
| `setup_controls(self)` | 绑定键盘和鼠标事件到控制输入状态。 | None | None |
| `update_controls(self, task)` | 任务：平滑更新控制面值（如 `self.throttle`），并传递给 `PhysicsManager`。 | `task`: Panda3D 任务对象。 | `Task.cont` |
| `toggle_view(self)` | 切换视角模式（第三人称、第一人称、自由视角）。 | None | None |
| `update_camera(self, task)` | 任务：根据当前视角模式更新相机的位置和朝向。 | `task`: Panda3D 任务对象。 | `Task.cont` |

## 3. `GameManager` (src/game_manager.py)

负责游戏逻辑、任务系统和计分机制。

| 方法 | 描述 | 参数 | 返回值 |
| :--- | :--- | :--- | :--- |
| `__init__(self, base, physics_mgr)` | 初始化检查点列表、分数和检查点模型。 | `base`: Panda3D `ShowBase` 实例。<br>`physics_mgr`: `PhysicsManager` 实例。 | None |
| `set_next_checkpoint(self)` | 设置下一个任务检查点的位置和视觉模型。 | None | None |
| `check_collision(self)` | 检查飞机是否通过了当前检查点，并更新分数和任务进度。 | None | None |
| `game_loop(self, task)` | 任务：主要的任务和逻辑循环。 | `task`: Panda3D 任务对象。 | `Task.cont` |
| `get_game_status(self)` | 返回当前游戏状态，供 HUD 显示。 | None | 包含 `score`, `current_checkpoint`, `total_checkpoints`, `mission_status` 的字典。 |

---
*作者：Manus AI*

