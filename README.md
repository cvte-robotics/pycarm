# pycarm

Python interface for cvte arm.

# Install

```
pip install carm
```

# Usage

```
import carm

arm = carm.Carm("ws://localhost:8090")

print("version:",carm.version)
print("limits:", carm.limit)
print("state:", carm.state)

carm.track_joint(carm.joint_pos)

carm.move_joint(carm.joint_pos)

carm.track_pose(carm.cart_pose)

carm.move_pose(carm.cart_pose)
```

# Version update to pypy

```
python3 -m build
python3 -m twine upload --repository pypi dist/*
```

# CARM Python SDK

本项目提供与 CARM 机械臂控制器通信的 Python 接口，基于 WebSocket 协议，封装了常用的控制命令和状态查询。支持单臂操作，可与 C++ SDK 功能对齐。

## 安装

### 依赖

* Python 3.6+
* websocket-client

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

## 快速开始

**python**

```
from carm import Carm

# 连接到机械臂（默认 IP: 10.42.0.101:8090）
robot = Carm("10.42.0.101")

# 等待连接成功，检查状态
if robot.is_connected():
    print("Connected!")

# 将机械臂设置为就绪状态（清除错误、上使能、位置模式）
robot.set_ready()

# 获取当前关节位置
print("Joint positions:", robot.joint_pos)

# 移动到目标关节位置（阻塞等待完成）
robot.move_joint([0.1, -0.2, 0.3, 0.0, 0.0, 0.0], is_sync=True)

# 关闭连接
robot.disconnect()
```

## API 参考

### 连接管理

#### `__init__(addr="10.42.0.101", arm_index=0)`

* 描述：初始化实例，自动连接指定 IP 的控制器。
* 参数：
  * `addr` (str): 控制器 IP 地址。
  * `arm_index` (int): 机械臂索引（0 表示第一个臂）。

#### `connect(addr=None, port=None, timeout=1)`

* 描述：连接到控制器。
* 参数：
  * `addr` (str, optional): 新 IP 地址。
  * `port` (int, optional): 新端口。
  * `timeout` (float): 超时秒数。
* 返回：`bool` 是否连接成功。

**python**

```
robot.connect(addr="192.168.1.100", timeout=2)
```

#### `disconnect()`

* 描述：断开连接。

#### `is_connected()`

* 描述：返回当前连接状态。
* 返回：`bool`

**python**

```
if robot.is_connected():
    print("Connected")
```

---

### 设备配置获取

#### `get_limits()`

* 描述：获取关节限位、最大速度、加速度等参数。
* 返回：配置数据的 JSON 响应。

#### `get_eeff_config()`

* 描述：获取末端执行器配置。
* 返回：配置数据的 JSON 响应。

---

### 状态属性（只读）

所有属性均从最新状态帧中提取，请确保已收到至少一次状态更新。

#### `version`

* 返回：控制器软件版本（字符串）。

**python**

```
print("Version:", robot.version)
```

#### `joint_pos` / `joint_vel` / `joint_tau`

* 返回：实际关节位置（弧度）、速度、力矩（列表，长度 = 自由度）。

**python**

```
print("Joint positions:", robot.joint_pos)
```

#### `plan_joint_pos` / `plan_joint_vel` / `plan_joint_tau`

* 返回：规划关节位置、速度、力矩。

#### `cart_pose`

* 返回：实际法兰位姿，格式 `[x, y, z, qw, qx, qy, qz]`。

#### `plan_cart_pose`

* 返回：规划法兰位姿。

#### `joint_external_tau` / `cart_external_force`

* 返回：关节外力矩、笛卡尔外力（6 维）。

#### 末端执行器属性

* `end_effector_state`: 状态（-1 未连接/无，0 未使能，1 正常）
* `end_effector_type`: 末端执行器类型
* `end_effector_name`: 末端执行器名称
* `end_effector_dof`: 末端执行器自由度
* `end_effector_pos` / `vel` / `tau`: 实际位置/速度/力矩（列表）
* `plan_end_effector_pos` / `vel` / `tau`: 规划值
* `gripper_state`: 夹爪状态（简化，-1/0/1）
* `gripper_pos` / `tau`: 夹爪位置和力矩（单值）
* `plan_gripper_pos` / `tau`: 规划夹爪值
* `hand_state`: 灵巧手状态
* `hand_pos` / `vel` / `tau`: 灵巧手实际位置/速度/力矩（列表）
* `plan_hand_pos` / `vel` / `tau`: 规划灵巧手值

**python**

```
print("Gripper position:", robot.gripper_pos)
```

#### `tool_index`

* 返回：当前工具号。

**python**

```
print("Current tool:", robot.tool_index)
```

---

### 控制命令

#### `set_ready(timeout_ms=3000)`

* 描述：将机械臂置为就绪状态（清除错误、伺服上使能、切换到位置模式）。
* 参数：
  * `timeout_ms` (int): 超时毫秒。
* 返回：`bool` 是否成功。

**python**

```
if robot.set_ready():
    print("Robot is ready")
```

#### `set_servo_enable(enable=True)`

* 描述：设置伺服使能。
* 参数：
  * `enable` (bool): True 上使能，False 下使能。

#### `set_control_mode(mode=1)`

* 描述：设置控制模式。
* 参数：
  * `mode` (int): 0-IDLE, 1-点位, 2-MIT, 3-拖动, 4-力位混合。

**python**

```
robot.set_control_mode(3)  # 进入拖动模式
```

#### `set_passthrough_data(mode, can_id, data)`

* 描述：设置透传数据。
* 参数：
  * `mode` (int): 模式。
  * `can_id` (int): CAN ID。
  * `data` (list/str): 透传数据（字节列表或十六进制字符串等，需底层支持）。
* 返回：对于 mode 1 或 2，成功时返回 `(can_id, bytes_data)` 元组；否则返回 JSON 请求响应信息。

**python**

```python
# 发送透传数据，data 可为列表或十六进制字符串
ret = robot.set_passthrough_data(mode=1, can_id=0x01, data=[0x0A, 0x0B])
print(ret)  # 成功示例输出: (1, b'\x0a\x0b')
```

#### `set_end_effector(dof, pos, vel, tau)`

* 描述：设置末端执行器（夹爪/灵巧手）的目标位置、速度、力矩。
* 参数：
  * `dof` (int): 自由度。
  * `pos` (float/list): 位置值或列表。
  * `vel` (float/list): 速度值或列表。
  * `tau` (float/list): 力矩值或列表。
* 说明：输入自动对齐到指定自由度，不足补零，超出截断。

**python**

```
# 单自由度夹爪
robot.set_end_effector(1, pos=0.02, vel=0.0, tau=5.0)
```

#### `set_gripper(pos, tau=10)`

* 描述：简化的夹爪控制（单自由度）。
* 参数：
  * `pos` (float): 夹爪间隔（米），范围 0~0.08。
  * `tau` (float): 夹持力矩（N·m），范围 0~100。

**python**

```
robot.set_gripper(0.03, tau=8)
```
#### `set_hand(pos, tau, vel)`

* 描述：设置灵巧手位置、力矩和速度。
* 参数：
  * `pos` (float/list): 灵巧手位置或列表。
  * `tau` (float/list): 灵巧手力矩或列表。
  * `vel` (float/list): 灵巧手速度或列表。
* 说明：输入自动对齐到指定自由度，不足补零，超出截断。

**python**

```python
# 设置 3 个自由度的灵巧手
robot.set_hand([0.1, 0.2, 0.3], tau=[5.0, 5.0, 5.0], vel=[0.1, 0.1, 0.1])
```
#### `set_tool_index(index)`

* 描述：切换当前工具号。
* 参数：
  * `index` (int): 工具索引。

**python**

```
robot.set_tool_index(1)
```

#### `get_tool_coordinate(tool)`

* 描述：获取指定工具坐标系（工具末端相对法兰的位姿）。
* 参数：
  * `tool` (int): 工具号。
* 返回：JSON 响应。

#### `set_collision_config(flag=True, level=10)`

* 描述：配置碰撞检测。
* 参数：
  * `flag` (bool): 是否开启。
  * `level` (int): 灵敏度等级 0~2（0 最灵敏）。

**python**

```
robot.set_collision_config(True, level=1)
```

#### `stop(type=0)`

* 描述：通用停止。
* 参数：
  * `type` (int): 0-暂停, 1-停止, 2-禁用, 3-紧急停止。

**python**

```
robot.stop(1)  # 停止
```

#### `stop_task(at_once=False)`

* 描述：停止当前任务。
* 参数：
  * `at_once` (bool): 是否立即停止（否则完成当前段后停止）。

#### `recover()`

* 描述：退出暂停/急停状态。

#### `clean_carm_error()`

* 描述：清除控制器错误。

#### `set_speed_level(level=5.0, response_level=20)`

* 描述：设置速度等级。
* 参数：
  * `level` (float): 0~10，对应 0%~100%。
  * `response_level` (int): 过渡周期数（1~10000）。

**python**

```
robot.set_speed_level(3.0, response_level=10)
```

---

### 运动接口

#### `track_joint(pos, end_effector=None)`

* 描述：关节空间轨迹跟踪（周期性发送目标关节位置）。
* 参数：
  * `pos` (list): 目标关节位置。
  * `end_effector` (float, optional): 夹爪目标位置（0~0.08）。

**python**

```
robot.track_joint([0.1, -0.2, 0.3, 0, 0, 0], end_effector=0.02)
```

#### `track_pose(pos, end_effector=None)`

* 描述：笛卡尔空间轨迹跟踪（周期性发送目标位姿）。

**python**

```
target_pose = [0.5, 0.0, 0.3, 0.707, 0.0, 0.707, 0.0]
robot.track_pose(target_pose, end_effector=0.02)
```

#### `move_joint(pos, tm=-1, is_sync=True, tool=0)`

* 描述：关节空间点到点运动（TASK_MOVJ）。
* 参数：
  * `pos` (list): 目标关节位置。
  * `tm` (float): 期望运动时间（-1 表示自动）。
  * is_sync (bool): 是否阻塞等待完成。
  * `tool` (int): 工具号。

**python**

```
res = robot.move_joint([0.2, -0.3, 0.4, 0, 0, 0], is_sync=True)
```

#### `move_pose(pos, tm=-1, is_sync=True, tool=0)`

* 描述：笛卡尔空间点到点运动。

#### `move_line_pose(pos, is_sync=True, tool=0)`

* 描述：笛卡尔直线运动（TASK_MOVL）。

**python**

```
robot.move_line_pose([0.6, 0.1, 0.3, 0.707, 0, 0.707, 0], is_sync=True)
```

#### `move_line_joint(pos, is_sync=True, tool=0)`

* 描述：关节空间直线运动。

#### `move_flow_pose(target_pos, line_theta_weight=0.5, accuracy=0.0001, is_sync=True, tool=0)`

* 描述：笛卡尔雅可比迭代运动（TASK_FLOW）。
* 参数：
  * `target_pos` (list): 目标位姿。
  * `line_theta_weight` (float): 位置/姿态权重（0~1）。
  * `accuracy` (float): 收敛精度。

**python**

```
robot.move_flow_pose([0.6, 0.1, 0.3, 0.707, 0, 0.707, 0], accuracy=0.001)
```

#### `move_toppra(targets, speed=100, tool=0, is_joint_val=True, is_sync=True)`

* 描述：基于 TOPPRA 的多点轨迹运动。
* 参数：
  * `targets` (list): 目标轨迹点列表（也可传入单个目标点），可以为关节位置序列或笛卡尔位姿序列。
  * `speed` (float): 速度百分比。
  * `tool` (int): 工具号。
  * `is_joint_val` (bool): True 表示关节空间目标，False 表示笛卡尔空间目标。
  * `is_sync` (bool): 是否阻塞等待完成。

**python**

```
robot.move_toppra([[0.1, -0.2, 0.3, 0, 0, 0], [0.2, -0.3, 0.4, 0, 0, 0]], is_joint_val=True)
```

#### `move_joint_traj(target_traj, gripper_pos=None, stamps=None, is_sync=True)`

* 描述：关节轨迹连续运动（内部通过 `move_toppra` 实现）。
* 参数：
  * `target_traj` (list): 目标关节位置轨迹列表。
  * `is_sync` (bool): 是否阻塞等待完成。
  * *(其他参数为兼容性预留，当前暂不生效)*

#### `move_pose_traj(target_traj, gripper_pos=None, stamps=None, is_sync=True)`

* 描述：位姿轨迹连续运动（内部通过 `move_toppra` 实现）。
* 参数：
  * `target_traj` (list): 目标笛卡尔位姿轨迹列表。
  * `is_sync` (bool): 是否阻塞等待完成。
  * *(其他参数为兼容性预留，当前暂不生效)*

---

### 示教接口

#### `trajectory_teach(off_on, name="")`

* 描述：开始/停止示教录制。
* 参数：
  * `off_on` (bool): True 开始，False 停止。
  * `name` (str): 轨迹名称。

**python**

```
robot.trajectory_teach(True)
# ... 移动机械臂 ...
robot.trajectory_teach(False, "my_traj_001")
```

#### `trajectory_recorder(name, is_sync=True)`

* 描述：复现指定名称的轨迹。

**python**

```
robot.trajectory_recorder("my_traj_001", is_sync=True)
```

#### `check_teach()`

* 描述：获取已录制的轨迹列表。
* 返回：列表。

**python**

```
traj_list = robot.check_teach()
print("Recorded trajectories:", traj_list)
```

---

### 运动学

#### `inverse_kine(cart_pose, ref_joints, tool=0)`

* 描述：逆运动学求解。
* 参数：
  * `cart_pose` (list 或 list of lists): 目标位姿（单个或多个）。
  * `ref_joints` (list 或 list of lists): 参考关节角。
  * `tool` (int): 工具号。
* 返回：包含关节角的 JSON 响应。

**python**

```
res = robot.inverse_kine([0.5, 0, 0.3, 0.707, 0, 0.707, 0], [0,0,0,0,0,0])
joints = res["data"]["joint1"]
```

#### `forward_kine(joint_pos, tool=0)`

* 描述：正运动学求解。
* 参数：
  * `joint_pos` (list 或 list of lists): 关节角。
  * `tool` (int): 工具号。
* 返回：位姿（单个或列表），失败返回 None。

**python**

```
pose = robot.forward_kine([0.1, -0.2, 0.3, 0, 0, 0])
print("Cartesian pose:", pose)
```

---

### 回调注册

#### `on_error(callback)`

* 描述：注册错误处理回调。
* 参数：
  * `callback`: 函数签名 `fn(error_code, error_message)`。

**python**

```
def my_error_handler(code, msg):
    print(f"Error {code}: {msg}")

robot.on_error(my_error_handler)
```

#### `on_task_finish(callback)`

* 描述：注册任务完成回调。
* 参数：
  * `callback`: 函数签名 `fn(task_key)`。

**python**

```
def task_done(task_key):
    print(f"Task {task_key} finished")

robot.on_task_finish(task_done)
```

---

## 注意事项

* 所有请求都是同步阻塞的，除非 `is_sync=False` 的运动接口。
* 状态属性（如 `joint_pos`）需在连接并收到状态更新后才能使用。
* 某些高级功能（如 PVT 轨迹）尚未实现，但占位已预留。

## 许可证

[MIT License](https://license/)
