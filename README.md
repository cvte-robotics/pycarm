# pycarm

Python interface for cvte arm.

本包提供三层接口：

- **`Carm`** — 底层 WebSocket 内核（`carm_kernel.py`），基于属性和简单回调的轻量级接口。
- **`CArmSingleCol`** — 单臂包装类（`carm.py`），方法签名与返回值与 C++ `CArmSingleCol`（`carm_cobot.h`）完全对齐。
- **`CArmDualBot`** — 双臂包装类（`carm.py`），组合两个 `CArmSingleCol`，方法签名与 C++ `CArmDualBot`（`carm_dual.h`）完全对齐。

# Install

```
pip install carm
```

# Usage

## 快速开始 — CArmSingleCol（对齐 C++ 单臂接口）

```python
from carm import CArmSingleCol

arm = CArmSingleCol("10.42.0.101")
arm.connect()
arm.set_ready()

# 命令类方法返回 int：1 成功，<1 失败
ret = arm.move_joint([0, 0, 0, 0, 0, 0], is_sync=True)
if ret == 1:
    print("运动完成")

# 查询类方法直接返回数据
print("关节位置:", arm.get_joint_pos())
print("末端位姿:", arm.get_cart_pose())

arm.disconnect()
```

## 快速开始 — CArmDualBot（对齐 C++ 双臂接口）

```python
from carm import CArmDualBot

dual = CArmDualBot("10.42.0.101")
dual.connect()
dual.set_ready()

# 左臂运动
dual.move_left_joint([0, 0, 0, 0, 0, 0, 0])
# 右臂运动
dual.move_right_joint([0, 0, 0, 0, 0, 0, 0])

# 获取左右臂状态
print("左臂关节位置:", dual.get_left_joint_pos())
print("右臂关节位置:", dual.get_right_joint_pos())

dual.disconnect()
```

# Version update to pypy

```
python3 -m build
python3 -m twine upload --repository pypi dist/*
```

# CARM Python SDK

本项目提供与 CARM 机械臂控制器通信的 Python 接口，基于 WebSocket 协议，封装了常用的控制命令和状态查询。

## 接口层级

| 类名              | 文件               | 对齐 C++ 头文件  | 说明                                                         |
| ----------------- | ------------------ | ---------------- | ------------------------------------------------------------ |
| `Carm`          | `carm_kernel.py` | —               | 底层 WebSocket 内核，供上层接口调用                          |
| `CArmSingleCol` | `carm.py`        | `carm_cobot.h` | 单臂包装类，方法签名与返回值约定与 C++ 完全对齐              |
| `CArmDualBot`   | `carm.py`        | `carm_dual.h`  | 双臂包装类，组合两个 `CArmSingleCol`，共享方法同时操作两臂 |

### 返回值约定

- **命令类方法**返回 `int`：`1` 表示成功，`<1` 表示失败（与 C++ 一致）。
- **查询类方法**返回对应数据类型（`list` / `float` / `dict` / `str` 等）。
- **输出参数**通过传入可变容器（`list` / `dict`）就地填充，匹配 C++ 引用语义。

### CArmDualBot 命名规则

`CArmDualBot` 的单臂方法采用 `{verb}_{side}_{noun}` 命名，与 C++ 完全一致：

- 查询类：`get_left_joint_pos`、`get_right_cart_pose`、`get_left_gripper_state`
- 控制类：`move_left_joint`、`track_right_pose`、`set_left_gripper`
- 回调类：`register_left_joint_cbk`、`release_right_pose_cbk`
- 底层类：`low_left_pv_command`、`low_right_set_robot_mode`
- 运动学类：`inverse_kine_left`、`forward_kine_right_array`
- 示教类：`trajectory_teach_left`、`trajectory_recorder_right`

共享方法（如 `connect`、`set_ready`、`set_speed_level`、`register_error_cbk` 等）同时操作两个臂，全部成功才返回 1。

### 兼容性

`from carm import Carm` 依然可用（兼容旧代码，不推荐），同时新增 `CArmSingleCol` 和 `CArmDualBot` 导出。

## 安装

### 依赖

- Python 3.6+
- websocket-client

### 安装方式

**bash**

```
pip install carm
```

或直接复制 `carm.py` 到您的项目目录，并安装依赖：

**bash**

```
pip install websocket-client
```

## API 参考

### 连接管理

#### `__init__(addr="10.42.0.101", arm_index=0)`

- 描述：初始化实例，自动连接指定 IP 的控制器。
- 参数：
  - `addr` (str): 控制器 IP 地址。
  - `arm_index` (int): 机械臂索引（0 表示第一个臂）。

#### `connect(addr=None, port=None, timeout=1)`

- 描述：连接到控制器。
- 参数：
  - `addr` (str, optional): 新 IP 地址。
  - `port` (int, optional): 新端口。
  - `timeout` (float): 超时秒数。
- 返回：`bool` 是否连接成功。

**python**

```
robot.connect(addr="192.168.1.100", timeout=2)
```

#### `disconnect()`

- 描述：断开连接。

#### `is_connected()`

- 描述：返回当前连接状态。
- 返回：`bool`

**python**

```
if robot.is_connected():
    print("Connected")
```

---

### 设备配置获取

#### `get_limits()`

- 描述：获取关节限位、最大速度、加速度等参数。
- 返回：字典，包含以下配置字段（可能依控制器的不同部分存在）：
  - `limit_lower` (list): 关节下限位 (rad)
  - `limit_upper` (list): 关节上限位 (rad)
  - `limit_vel` (list): 关节最大速度 (rad/s)
  - `limit_acc` (list): 关节最大加速度 (rad/s^2)
  - `limit_jerk` (list): 关节最大加加速度 (rad/s^3)

#### `get_eeff_config()`

- 描述：获取末端执行器配置。
- 返回：包含完整末端配置特性的字典，使用时请通过以下键名解析：
  - `eeff_dof` (int): 末端执行器自由度数量
  - `eeff_lower` (list): 末端下限位
  - `eeff_upper` (list): 末端上限位
  - `eeff_vel` (list): 末端最大速度
  - `eeff_tau` (list): 末端最大力矩

---

### 状态属性（只读）

所有属性均从最新状态帧中提取，请确保已收到至少一次状态更新。

#### 机械臂基础状态

- `arm_name`: 机械臂名称（字符串）
- `arm_dof`: 机械臂自由度（整数）
- `servo_status`: 伺服状态（1: 使能，0: 失能）
- `controller_state`: 控制器运行状态（-1-error, 0-standby, 1-running, 2-dragging）
- `fsm_mode`: 控制器模式（如 ERROR = -1, IDLE, POSITION, MIT, CURRENT, PF, TELEOPERATION）
- `speed_percentage`: 获取当前设定运行速度标幺值
- `on_debug_mode`: 是否在仿真状态（布尔值）

#### `version`

- 返回：控制器软件版本（字符串）。

**python**

```
print("Version:", robot.version)
```

#### `joint_pos` / `joint_vel` / `joint_tau`

- 返回：实际关节位置（弧度）、速度、力矩（列表，长度 = 自由度）。

**python**

```
print("Joint positions:", robot.joint_pos)
```

#### `plan_joint_pos` / `plan_joint_vel` / `plan_joint_tau`

- 返回：规划关节位置、速度、力矩。

#### `cart_pose`

- 返回：实际法兰位姿，格式 `[x, y, z, qw, qx, qy, qz]`。

#### `plan_cart_pose`

- 返回：规划法兰位姿。

#### `joint_external_tau` / `cart_external_force`

- 返回：关节外力矩、笛卡尔外力（6 维）。

#### 末端执行器属性

- `end_effector_state`: 状态（-1 未连接/无，0 未使能，1 正常）
- `end_effector_type`: 末端执行器类型
- `end_effector_name`: 末端执行器名称
- `end_effector_dof`: 末端执行器自由度
- `end_effector_pos` / `vel` / `tau`: 实际位置/速度/力矩（列表）
- `plan_end_effector_pos` / `vel` / `tau`: 规划值
- `gripper_state`: 夹爪状态（简化，-1/0/1）
- `gripper_pos` / `tau`: 夹爪位置和力矩（单值）
- `plan_gripper_pos` / `tau`: 规划夹爪值
- `hand_state`: 灵巧手状态
- `hand_pos` / `vel` / `tau`: 灵巧手实际位置/速度/力矩（列表）
- `plan_hand_pos` / `vel` / `tau`: 规划灵巧手值

**python**

```
print("Gripper position:", robot.gripper_pos)
```

#### `tool_index`

- 返回：当前工具号。

**python**

```
print("Current tool:", robot.tool_index)
```

---

### 控制命令

> **注：** 大部分运动控制和配置指令在执行成功且被服务端接收时，默认返回布尔值 `True`，因异常或拒绝时返回 `False`。

#### `set_ready(timeout_ms=3000)`

- 描述：将机械臂置为就绪状态（清除错误、伺服上使能、切换到位置模式）。
- 参数：
  - `timeout_ms` (int): 超时毫秒。
- 返回：`bool` 是否成功。

**python**

```
if robot.set_ready():
    print("Robot is ready")
```

#### `set_servo_enable(enable=True)`

- 描述：设置伺服使能。
- 参数：
  - `enable` (bool): True 上使能，False 下使能。

#### `set_control_mode(mode=1)`

- 描述：设置控制模式。
- 参数：
  - `mode` (int): 0-IDLE, 1-点位, 2-MIT, 3-拖动, 4-力位混合。

**python**

```
robot.set_control_mode(3)  # 进入拖动模式
```

#### `set_passthrough_data(mode, can_id, data)`

- 描述：设置透传数据。
- 参数：
  - `mode` (int): 模式，0-仅发送，1-仅接收，2-发送并接收。
  - `can_id` (int): CAN ID。
  - `data` (list/str): 透传数据（字节列表或十六进制字符串等，需底层支持）。
- 返回：当模式为 1 或 2 且执行成功时，返回 `(True, can_id, bytes_data)` 三元组；其它情况返回 `(False, None, None)`。

**python**

```python
# 发送透传数据，data 可为列表或十六进制字符串
success, can_id, data = robot.set_passthrough_data(mode=1, can_id=0x01, data=[0x0A, 0x0B])
print(success, can_id, data)  # 成功示例输出: True 1 b'\x0a\x0b'
```

#### `set_end_effector(dof, pos, vel, tau)`

- 描述：设置末端执行器（夹爪/灵巧手）的目标位置、速度、力矩。
- 参数：
  - `dof` (int): 自由度。
  - `pos` (float/list): 位置值或列表。
  - `vel` (float/list): 速度值或列表。
  - `tau` (float/list): 力矩值或列表。
- 说明：输入自动对齐到指定自由度，不足补零，超出截断。

**python**

```
# 单自由度夹爪
robot.set_end_effector(1, pos=0.02, vel=0.0, tau=5.0)
```

#### `set_gripper(pos, tau=10)`

- 描述：简化的夹爪控制（单自由度）。
- 参数：
  - `pos` (float): 夹爪间隔（米），范围 0~0.08。
  - `tau` (float): 夹持力矩（N·m），范围 0~100。

**python**

```
robot.set_gripper(0.03, tau=8)
```

#### `set_hand(pos, tau, vel)`

- 描述：设置灵巧手位置、力矩和速度。
- 参数：
  - `pos` (float/list): 灵巧手位置或列表。
  - `tau` (float/list): 灵巧手力矩或列表。
  - `vel` (float/list): 灵巧手速度或列表。
- 说明：输入自动对齐到指定自由度，不足补零，超出截断。

**python**

```python
# 设置 3 个自由度的灵巧手
robot.set_hand([0.1, 0.2, 0.3], tau=[5.0, 5.0, 5.0], vel=[0.1, 0.1, 0.1])
```

#### `set_tool_index(index)`

- 描述：切换当前工具号。
- 参数：
  - `index` (int): 工具索引。

**python**

```
robot.set_tool_index(1)
```

#### `get_tool_coordinate(tool)`

- 描述：获取指定工具坐标系（工具末端相对法兰的位姿）。
- 参数：
  - `tool` (int): 工具号。
- 返回：工具的笛卡尔坐标 `[x, y, z, qx, qy, qz, qw]` 列表，失败或异常返回空列表 `[]`。

#### `set_collision_config(flag=True, level=0)`

- 描述：配置碰撞检测。
- 参数：
  - `flag` (bool): 是否开启。
  - `level` (int): 灵敏度等级 0~2（0 最灵敏）。

**python**

```
robot.set_collision_config(True, level=1)
```

#### `stop(type=0)`

- 描述：通用停止。
- 参数：
  - `type` (int): 0-暂停, 1-停止, 2-禁用, 3-紧急停止。

**python**

```
robot.stop(1)  # 停止
```

#### `stop_task(at_once=False)`

- 描述：停止当前任务。
- 参数：
  - `at_once` (bool): 是否立即停止（否则完成当前段后停止）。

#### `recover()`

- 描述：退出暂停/急停状态。

#### `clean_carm_error()`

- 描述：清除控制器错误。

#### `set_speed_level(level=5.0, response_level=20)`

- 描述：设置速度等级。
- 参数：
  - `level` (float): 0~10，对应 0%~100%。
  - `response_level` (int): 过渡周期数（1~10000）。

**python**

```
robot.set_speed_level(3.0, response_level=10)
```

---

### 运动接口

#### `track_joint(pos, end_effector=None)`

- 描述：关节空间轨迹跟踪（周期性发送目标关节位置）。
- 参数：
  - `pos` (list): 目标关节位置。
  - `end_effector` (float, optional): 夹爪目标位置（0~0.08）。

**python**

```
robot.track_joint([0.1, -0.2, 0.3, 0, 0, 0], end_effector=0.02)
```

#### `track_pose(pos, end_effector=None)`

- 描述：笛卡尔空间轨迹跟踪（周期性发送目标位姿）。

**python**

```
target_pose = [0.5, 0.0, 0.3, 0.707, 0.0, 0.707, 0.0]
robot.track_pose(target_pose, end_effector=0.02)
```

#### `move_joint(pos, desire_time=-1, is_sync=True, tool=0)`

- 描述：关节空间点到点运动（TASK_MOVJ）。
- 参数：
  - `pos` (list): 目标关节位置。
  - `desire_time` (float): 期望运动时间（-1 表示自动）。
  - `is_sync` (bool): 是否阻塞等待完成。
  - `tool` (int): 工具号。

**python**

```
res = robot.move_joint([0.2, -0.3, 0.4, 0, 0, 0], is_sync=True)
```

#### `move_pose(pos, desire_time=-1, is_sync=True, tool=0)`

- 描述：笛卡尔空间点到点运动。

#### `move_line_pose(pos, is_sync=True, tool=0)`

- 描述：笛卡尔直线运动（TASK_MOVL）。

**python**

```
robot.move_line_pose([0.6, 0.1, 0.3, 0.707, 0, 0.707, 0], is_sync=True)
```

#### `move_line_joint(pos, is_sync=True, tool=0)`

- 描述：关节空间直线运动。

#### `move_flow_pose(target_pos, line_theta_weight=0.5, accuracy=0.0001, move_line=False, is_sync=True, tool=0)`

- 描述：笛卡尔雅可比迭代运动（TASK_FLOW）。
- 参数：
  - `target_pos` (list): 目标位姿。
  - `line_theta_weight` (float): 位置/姿态权重（0~1）。
  - `accuracy` (float): 收敛精度。
  - `move_line` (bool): 是否直线运动。

**python**

```
robot.move_flow_pose([0.6, 0.1, 0.3, 0.707, 0, 0.707, 0], accuracy=0.001)
```

#### `move_toppra(targets, speed=100, tool=0, is_joint_val=True, is_sync=True)`

- 描述：基于 TOPPRA 的多点轨迹运动。
- 参数：
  - `targets` (list): 目标轨迹点列表（也可传入单个目标点），可以为关节位置序列或笛卡尔位姿序列。
  - `speed` (float): 速度百分比。
  - `tool` (int): 工具号。
  - `is_joint_val` (bool): True 表示关节空间目标，False 表示笛卡尔空间目标。
  - `is_sync` (bool): 是否阻塞等待完成。

**python**

```
robot.move_toppra([[0.1, -0.2, 0.3, 0, 0, 0], [0.2, -0.3, 0.4, 0, 0, 0]], is_joint_val=True)
```

#### `move_joint_traj(target_traj, gripper_pos=None, stamps=None, is_sync=True)`

- 描述：关节轨迹连续运动。`stamps` 为空时使用 TOPPRA 路径规划，有值时使用 PVT 模式。
- 参数：
  - `target_traj` (list): 目标关节位置轨迹列表。
  - `gripper_pos` (list, optional): 夹爪位置列表（仅 PVT 模式生效），与路点一一对应。
  - `stamps` (list, optional): 时间戳列表（秒），None 或空时使用 TOPPRA，有值时使用 PVT。
  - `is_sync` (bool): 是否阻塞等待完成。

#### `move_pose_traj(target_traj, gripper_pos=None, stamps=None, is_sync=True)`

- 描述：位姿轨迹连续运动。`stamps` 为空时使用 TOPPRA 路径规划，有值时使用 PVT 模式。
- 参数：
  - `target_traj` (list): 目标笛卡尔位姿轨迹列表。
  - `gripper_pos` (list, optional): 夹爪位置列表（仅 PVT 模式生效）。
  - `stamps` (list, optional): 时间戳列表（秒），None 或空时使用 TOPPRA，有值时使用 PVT。
  - `is_sync` (bool): 是否阻塞等待完成。

#### `move_pvt(target_pos, gripper_pos=None, stamps=None, is_joint_val=True, is_sync=True)`

- 描述：PVT（位置-速度-时间）模式运动，无需加入起始点，直接给目标点。
- 参数：
  - `target_pos` (list): 规划路点列表，每个路点为关节角或位姿。
  - `gripper_pos` (list, optional): 夹爪位置列表，与路点一一对应，None 或空为不运动夹爪。
  - `stamps` (list): 时间戳列表（秒），与路点一一对应，必须递增且大于0。
  - `is_joint_val` (bool): True 表示关节空间目标，False 表示笛卡尔空间目标。
  - `is_sync` (bool): 是否同步等待任务完成。

**python**

```python
waypoints = [[0.1, -0.2, 0.3, 0, 0, 0], [0.2, -0.3, 0.4, 0, 0, 0]]
stamps = [2.0, 4.0]
robot.move_pvt(waypoints, stamps=stamps, is_joint_val=True)
```

---

### 示教接口

#### `trajectory_teach(off_on, name="")`

- 描述：开始/停止示教录制。
- 参数：
  - `off_on` (bool): True 开始，False 停止。
  - `name` (str): 轨迹名称。

**python**

```
robot.trajectory_teach(True)
# ... 移动机械臂 ...
robot.trajectory_teach(False, "my_traj_001")
```

#### `trajectory_recorder(name, is_sync=True)`

- 描述：复现指定名称的轨迹。

**python**

```
robot.trajectory_recorder("my_traj_001", is_sync=True)
```

#### `check_teach()`

- 描述：获取已录制的轨迹列表。
- 返回：列表。

**python**

```
traj_list = robot.check_teach()
print("Recorded trajectories:", traj_list)
```

---

### 运动学

#### `inverse_kine(cart_pose, ref_joints, tool=0)`

- 描述：逆运动学求解。
- 参数：
  - `cart_pose` (list 或 list of lists): 目标位姿（单个或多个）。
  - `ref_joints` (list 或 list of lists): 参考关节角。
  - `tool` (int): 工具号。
- 返回：成功时返回解析过的关节角结果（单个一维列表或二维列表）。失败或异常返回空列表 `[]`。

**python**

```python
joints = robot.inverse_kine([0.5, 0, 0.3, 0.707, 0, 0.707, 0], [0,0,0,0,0,0])
print("Inverse joints:", joints)
```

#### `forward_kine(joint_pos, tool=0)`

- 描述：正运动学求解。
- 参数：
  - `joint_pos` (list 或 list of lists): 关节角。
  - `tool` (int): 工具号。
- 返回：成功时返回解析过的位姿结果（单个一维列表或二维列表）。失败或异常返回空列表 `[]`。

**python**

```
pose = robot.forward_kine([0.1, -0.2, 0.3, 0, 0, 0])
print("Cartesian pose:", pose)
```

---

### 工具与诊断接口

#### `ping(date=None)`

- 描述：发送 ping 请求，用于测试通讯延时及带宽。
- 参数：
  - `date` (str, optional): 携带的数据载荷。
- 返回：`dict`，包含 `Unix_resp`（接收时间戳）和 `data`（载荷）。

#### `set_debug(flag=False)`

- 描述：设置控制器进入 debug 仿真模式，该模式下不连接机械臂。
- 参数：
  - `flag` (bool): True 开启仿真模式，False 关闭。
- 返回：`bool` 执行是否成功。

---

### 底层透传接口（Low-Level）

底层透传接口提供伺服级的高速控制能力，适用于需要 1ms 级控制周期的场景。使用前需调用 `set_low_mode(True)` 进入底层模式。

#### `set_low_mode(flag=True)`

- 描述：设置底层透传模式。
- 参数：
  - `flag` (bool): True 开启，False 关闭。
- 返回：`bool`

#### 底层控制指令

以下指令均返回 `(success: bool, low_state: dict)` 元组。执行后可通过 `@property` 获取最新底层状态。

##### `low_pv_command(pos, vel)`

- 描述：发送底层位置速度(PV)控制指令。
- 参数：`pos` (list) 目标关节位置 (rad)，`vel` (list) 目标关节速度 (rad/s)。

##### `low_mit_command(pos, vel, tau, kp, kd)`

- 描述：发送底层 MIT 综合控制指令。
- 参数：`pos`/`vel`/`tau`/`kp`/`kd` 均为 list，分别对应关节位置、速度、前馈力矩、刚度、阻尼。

##### `low_pf_command(pos, vel, tau)`

- 描述：发送底层位置力矩(PF)混合控制指令。

##### `low_current_command(tau)`

- 描述：发送底层力矩(Current)指令。
- 参数：`tau` (list) 目标关节力矩 (N·m)。

##### `low_refresh()`

- 描述：主动刷新并获取底层硬件数据（无需下发控制指令）。

##### `low_set_end_effector_ctr(pos, vel, tau)`

- 描述：底层末端执行器控制指令。

#### 底层配置指令

##### `low_set_robot_mode(mode)`

- 描述：设置机器人底层运行模式。

##### `low_set_end_effector_mode(mode)`

- 描述：设置末端执行器底层运行模式。

##### `low_set_servo_enable(status)`

- 描述：控制底层伺服上/下使能。
- 参数：`status` (bool) True 上使能，False 下使能。

##### `low_reset(cnt=5)`

- 描述：进行底层错误复位操作。
- 参数：`cnt` (int) 尝试复位的最大次数。

#### 底层状态查询

##### `low_get_servo_status()`

- 描述：主动获取底层伺服级状态。
- 返回：`dict`，包含 `mitKp`/`mitKd`/`pvVel`/`pfVel`/`isServoEnable`/`fsmMode`/`isConnected`/`mosTemperature`/`motorTemperature`/`motorVBus`/`motorErrorCode`/`motorErrorMsg` 等字段。

##### `low_get_inverse_kine(pose, refer_pos, tool=-1)`

- 描述：执行底层逆运动学闭式求解。
- 返回：`(success, tool, joint_pos)` 元组。

##### `low_get_forward_kine(joint_pos, tool=-1)`

- 描述：执行底层正运动学计算。
- 返回：`(success, tool, pose)` 元组。

##### `low_get_dynamics(joint_pos, joint_vel, joint_acc)`

- 描述：计算惯性矩阵(M)、科里奥利力(C)、重力(G)分量。
- 返回：`(success, tool, m_force, c_force, g_force)` 元组。

##### `low_get_jacobian(joint_pos)`

- 描述：获取底层雅可比矩阵。
- 返回：`(success, tool, matrix)` 元组，`matrix` 为二维列表 `list[list[float]]`，形状 `(rows, cols)`。

##### `low_get_nullspace(joint_pos, tolerance)`

- 描述：获取底层零空间矩阵。
- 返回：`(success, tool, matrix)` 元组，`matrix` 为二维列表。

#### 底层状态属性（@property）

执行底层控制指令后，可通过以下属性直接读取最新底层状态：

**臂状态（来自 RobotStatus）：**

- `low_error_code` / `low_error_msg`: 错误码和信息
- `low_arm_connected` / `low_arm_enable`: 连接和使能状态
- `low_arm_status` / `low_arm_mode`: 臂状态和模式

**关节状态（来自 RobotState）：**

- `low_joint_cmd_pos` / `low_joint_cmd_vel` / `low_joint_cmd_tau`: 指令值
- `low_joint_pos` / `low_joint_vel` / `low_joint_tau`: 实际值

**夹爪状态（来自 gripperStatus / gripperState）：**

- `low_gripper_connected` / `low_gripper_enable` / `low_gripper_err_code` / `low_gripper_mode`
- `low_gripper_cmd_pos` / `low_gripper_cmd_vel` / `low_gripper_cmd_tau`
- `low_gripper_pos` / `low_gripper_vel` / `low_gripper_tau`

**python**

```python
robot.set_low_mode(True)
robot.low_set_servo_enable(True)
robot.low_set_robot_mode(1)

# PV 控制循环
ok, state = robot.low_pv_command(target_pos, target_vel)
print("实际关节位置:", robot.low_joint_pos)
print("实际关节力矩:", robot.low_joint_tau)
print("臂连接状态:", robot.low_arm_connected)

robot.low_set_servo_enable(False)
robot.set_low_mode(False)
```

---

### 回调注册

#### `on_error(callback)`

- 描述：注册错误处理回调。
- 参数：
  - `callback`: 函数签名 `fn(error_info)`，其中 `error_info` 是包含错误详情的字典。

**python**

```
def my_error_handler(error_info):
    print(f"Error: {error_info}")

robot.on_error(my_error_handler)
```

#### `on_task_finish(callback)`

- 描述：注册任务完成回调。
- 参数：
  - `callback`: 函数签名 `fn(task_key)`。

**python**

```
def task_done(task_key):
    print(f"Task {task_key} finished")

robot.on_task_finish(task_done)
```

#### `on_update(callback)`

- 描述：注册状态更新回调。
- 参数：
  - `callback`: 函数签名 `fn(Unix_time)`，参数为当前状态更新的 Unix 时间戳（Unix_time_stamp，double类型）。

**python**

```python
def state_updated(timestamp):
    print(f"State updated at timestamp: {timestamp}")

robot.on_update(state_updated)
```

---

## 注意事项

- 所有请求都是同步阻塞的，除非 `is_sync=False` 的运动接口。
- 状态属性（如 `joint_pos`）需在连接并收到状态更新后才能使用。
- 底层透传接口（`low_*`）使用前需先调用 `set_low_mode(True)` 进入底层模式，使用完毕后调用 `set_low_mode(False)` 退出。
- 底层控制指令（`low_pv_command` 等）会自动对输入进行非法值检查、维度校验和限幅处理。

## 许可证

[MIT License](https://license/)
