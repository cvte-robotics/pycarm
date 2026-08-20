import websocket
import threading
import json
import logging
import uuid
import time
import math

_LOGGER = logging.getLogger(__name__)

ECAT_CAN_FLAG_EXTENDED = 1 << 0
ECAT_CAN_FLAG_REMOTE = 1 << 1
ECAT_CAN_FLAG_BRS = 1 << 3
ECAT_CAN_FLAG_ESI = 1 << 4

class Carm:
    def __init__(self, addr="10.42.0.101", arm_index=0, port=8090):
        self.state = None
        self.low_state_data = None
        self.last_msg = None
        self.ws = None
        self.arm_index = arm_index
        self.addr = addr
        self.port = port
        self._retry_delay = 1.0
        self._max_delay = 10.0

        self.ops = {
            "webSendRobotState": lambda msg: self.__cbk_status(msg)
        }

        self.call_back = {
            "taskFinished": lambda msg: self.__cbk_taskfinish(msg),
            "onCarmError": lambda msg: self.__cbk_error(msg),
            "updateRobotState": lambda msg: self.__cbk_update(msg)
        }

        self.res_pool = {}
        self.task_pool = {}
        self.open_ready = threading.Event()
        self._reconnect_event = threading.Event()
        self.limit = None
        self.eeff_limit = None
        self._running = True
        self._create_connection() # 确保第一次的 ws 对象先创建出来
        self.reader = threading.Thread(target=self.__recv_loop, daemon=True)
        self.reader.start()

    def __del__(self):
        self.disconnect()

    def _create_connection(self):
        """创建 WebSocket 连接（内部使用）"""
        self.ws = websocket.WebSocketApp(
            f"ws://{self.addr}:{self.port}",
            on_open=lambda ws: self.__on_open(ws),
            on_close=lambda ws, code, close_msg: self.__on_close(ws, code, close_msg),
            on_message=lambda ws, msg: self.__on_message(ws, msg),
        )
        print(f"Connecting to {self.addr}:{self.port}...")

    # -------------------- 连接管理 --------------------
    def connect(self, addr=None, port=None, timeout=1) -> bool:
        """
        连接到机械臂控制器。

        :param addr: str, IP 地址，默认使用初始化时的地址

        :param port: int, 端口，默认 8090

        :param timeout: float, 连接超时（秒），默认 1

        :return: bool, True 表示连接成功，False 表示失败
        """
        if addr:
            self.addr = addr
        if port:
            self.port = port
        self.disconnect()
        self._running = True
        self._retry_delay = 1.0
        self._reconnect_event.clear()
        self._create_connection()
        self.reader = threading.Thread(target=self.__recv_loop, daemon=True)
        self.reader.start()
        return self.open_ready.wait(timeout)

    def disconnect(self) -> None:
        """断开与机械臂控制器的连接，绝对安全地释放 ws 及停止后台线程"""
        if not getattr(self, '_running', False) and getattr(self, 'reader', None) is None:
            return 

        self._running = False
        self._reconnect_event.set() # 唤醒可能在沉睡（等待重连 delay）的旧线程
        
        # 暴力摧毁底层 socket 使得当前卡在 run_forever 的 receive 立即爆错抛出
        self.__force_close_ws_socket(getattr(self, 'ws', None))
        self.__abort_all_tasks()
        
        old_reader = getattr(self, 'reader', None)
        self.reader = None  # 剥离当前线程引用，通知旧线程必须退出
        
        # 必须死等该旧线程正式退出返回（杜绝它在任何地方死灰复燃产生僵尸分身）
        if old_reader and old_reader.is_alive() and old_reader is not threading.current_thread():
            old_reader.join(timeout=3.0) # timeout 3s 防止连接不正确的IP卡死，无timeout 绝对保证断联后后台干净
            
        self.open_ready.clear()
        self.ws = None # 完全切断残余连接对象引用
        print("Disconnected cleanly.")

    def is_connected(self) -> bool:
        """检查当前是否已连接"""
        return self.open_ready.is_set()

    # -------------------- 属性（状态获取） --------------------
    @property
    def _arm_state(self):
        """内部方法：安全获取当前手臂的状态字典"""
        if not self.state or "arm" not in self.state or not isinstance(self.state["arm"], list) or len(self.state["arm"]) <= self.arm_index:
            return {}
        arm_state = self.state["arm"][self.arm_index]
        # 专门处理新兼容旧协议
        if "eeff" not in arm_state or arm_state["eeff"].get("eeff_type") is None:
            dof = arm_state.get("eeff", {}).get("eeff_dof", 0)
            if dof == 1:
                arm_state["eeff"]["eeff_type"] = "gripper"
            elif dof == 6:
                arm_state["eeff"]["eeff_type"] = "hand"
            else:
                arm_state["eeff"]["eeff_type"] = "flange"

        if "eeff" not in arm_state or arm_state["eeff"].get("eeff_name") is None:
            dof = arm_state.get("eeff", {}).get("eeff_dof", 0)
            if dof == 1:
                arm_state["eeff"]["eeff_name"] = "gripper"
            elif dof == 6:
                arm_state["eeff"]["eeff_name"] = "hand"
            else:
                arm_state["eeff"]["eeff_name"] = "flange"

        return arm_state

    @property
    def arm_name(self):
        """机械臂名称"""
        return self._arm_state.get("arm_name", "")

    @property
    def arm_dof(self):
        """机械臂自由度"""
        return self._arm_state.get("arm_dof", 0)

    @property
    def servo_status(self):
        """伺服状态：1 使能，0 失能"""
        return self._arm_state.get("servo", 0)

    @property
    def controller_state(self):
        """控制器运行状态：-1-error, 0-standby, 1-running, 2-dragging"""
        return self._arm_state.get("state", 0)

    @property
    def fsm_mode(self):
        """控制器模式string：ERROR = -1, IDLE, POSITION, MIT, CURRENT, PF, TELEOPERATION, IMPEDANCE"""
        return self._arm_state.get("fsm_state", "IDLE")

    @property
    def speed_percentage(self):
        """当前设定运行速度标幺值（0.0~1.0）"""
        return self._arm_state.get("vel_per", 1.0)

    @property
    def on_debug_mode(self):
        """是否在仿真状态"""
        if not self.state:
            return False
        return self.state.get("on_debug_mode", False)

    @property
    def version(self) -> str:
        """
        获取控制器软件版本
        
        :return: str, 版本号字符串
        """
        res = self.request({
            "command": "getArmIntrinsicProperties",
            "arm_index": self.arm_index,
            "type": "version"
        })
        return res.get("version", "")

    def get_limits(self) -> dict:
        """
        获取关节限位、最大速度、加速度等参数。
        
        :return: dict 完整包含以下配置字段（可能依控制器的不同部分存在）：

                 - limit_lower: [list] 关节下限位 (rad)
                 - limit_upper: [list] 关节上限位 (rad)
                 - joint_vel: [list] 关节最大速度 (rad/s)
                 - joint_acc: [list] 关节最大加速度 (rad/s^2)
                 - joint_dec: [list] 关节最大减速度 (rad/s^2)
                 - joint_jerk: [list] 关节最大加加速度 (rad/s^3)
        """
        res = self.request({
            "command": "getJointParams",
            "arm_index": self.arm_index
        })
        return res.get("params", {})

    def get_eeff_config(self) -> dict:
        """
        获取末端执行器配置。
        
        :return: dict 完整包含以下末端配置字段：
        
                 - dof: [int] 末端执行器自由度数量
                 - eeff_name: [str] 末端执行器名称
                 - eeff_type: [str] 末端执行器类型
                 - eeff_lower: [list] 末端下限位
                 - eeff_upper: [list] 末端上限位
                 - eeff_vel: [list] 末端最大速度
                 - eeff_tau: [list] 末端最大力矩
        """
        res = self.request({
            "command": "getEeffParams",
            "arm_index": self.arm_index
        })
        if res.get("params") == None:
            if self.end_effector_type == "gripper":
                res["params"] = {
                    "dof": self.end_effector_dof,
                    "eeff_name": self.end_effector_name,
                    "eeff_type": self.end_effector_type,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [0.077]*self.end_effector_dof,
                    "eeff_vel": [0.0]*self.end_effector_dof,
                    "eeff_tau": [100.0]*self.end_effector_dof
                }
            elif self.end_effector_type == "hand":
                res["params"] = {
                    "dof": self.end_effector_dof,
                    "eeff_name": self.end_effector_name,
                    "eeff_type": self.end_effector_type,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [255.0]*self.end_effector_dof,
                    "eeff_vel": [255.0]*self.end_effector_dof,
                    "eeff_tau": [255.0]*self.end_effector_dof
                }
            else:
                res["params"] = {
                    "dof": self.end_effector_dof,
                    "eeff_name": self.end_effector_name,
                    "eeff_type": self.end_effector_type,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [255.0]*self.end_effector_dof,
                    "eeff_vel": [255.0]*self.end_effector_dof,
                    "eeff_tau": [255.0]*self.end_effector_dof
                }
        return res.get("params", {})

    @property
    def joint_pos(self):
        """实际关节位置 (rad)"""
        return self._arm_state.get("reality", {}).get("pose", [])

    @property
    def joint_vel(self):
        """实际关节速度 (rad/s)"""
        return self._arm_state.get("reality", {}).get("vel", [])

    @property
    def joint_tau(self):
        """实际关节力矩 (N·m)"""
        return self._arm_state.get("reality", {}).get("torque", [])

    @property
    def plan_joint_pos(self):
        """规划关节位置 (rad)"""
        return self._arm_state.get("plan", {}).get("pose", [])

    @property
    def plan_joint_vel(self):
        """规划关节速度 (rad/s)"""
        return self._arm_state.get("plan", {}).get("vel", [])

    @property
    def plan_joint_tau(self):
        """规划关节力矩 (N·m)"""
        return self._arm_state.get("plan", {}).get("torque", [])

    @property
    def cart_pose(self):
        """当前实际笛卡尔位姿 (x, y, z, qx, qy, qz, qw)"""
        return self._arm_state.get("pose", [])

    @property
    def plan_cart_pose(self):
        """当前规划笛卡尔位姿 (x, y, z, qx, qy, qz, qw)"""
        return self._arm_state.get("plan", {}).get("cart_pose", [])

    @property
    def joint_external_tau(self):
        """关节外力矩 (tau1~tauN)"""
        return self._arm_state.get("joint_external_tau", [])

    @property
    def cart_external_force(self):
        """笛卡尔外力 (fx, fy, fz, tx, ty, tz)"""
        return self._arm_state.get("cart_external_force", [])

    # -------------------- 末端执行器（夹爪）属性 --------------------
    @property
    def end_effector_state(self):
        """末端执行器状态（-1 未连接/无夹爪，0 未使能，1 正常）"""
        eeff = self._arm_state.get("eeff", {})
        if not eeff.get("is_connect", False):
            return -1
        return eeff.get("eeff_state", -1)

    @property
    def end_effector_pos(self):
        """实际末端执行器位置（单位：m or rad）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_pos", [])

    @property
    def end_effector_vel(self):
        """实际末端执行器速度（单位：m/s or rad/s）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_vel", [])

    @property
    def end_effector_tau(self):
        """实际末端执行器力矩（单位：N）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_tau", [])

    @property
    def end_effector_motor_pos(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_pos", [])

    @property
    def end_effector_motor_vel(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_vel", [])

    @property
    def end_effector_motor_tau(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_tau", [])

    @property
    def plan_end_effector_pos(self):
        """规划末端执行器位置（单位：m or rad）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_plan_pos", [])

    @property
    def plan_end_effector_vel(self):
        """规划末端执行器速度（单位：m/s or rad/s）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_plan_vel", [])

    @property
    def plan_end_effector_tau(self):
        """规划末端执行器力矩（单位：N）"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_plan_tau", [])

    @property
    def plan_end_effector_motor_pos(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_plan_pos", [])

    @property
    def plan_end_effector_motor_vel(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_plan_vel", [])

    @property
    def plan_end_effector_motor_tau(self):
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_motor_plan_tau", [])
    
    @property
    def end_effector_type(self):
        """末端执行器类型"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_type", "")
    
    @property
    def end_effector_name(self):
        """末端执行器名称"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_name", "")

    @property
    def end_effector_dof(self):
        """末端执行器自由度"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("eeff_dof", 0)

    @property
    def end_effector_is_connect(self):
        """末端执行器是否已连接"""
        eeff = self._arm_state.get("eeff", {})
        return eeff.get("is_connect", False)

    # -------------------- deprecated: 旧 gripper/hand 属性 --------------------
    @property
    def gripper_state(self):
        """夹爪状态（-1 未连接/无夹爪，0 未使能，1 正常）"""
        eeff = self._arm_state.get("eeff", {})
        if not eeff.get("is_connect", False) or self.end_effector_type != "gripper":
            return -1
        return self.end_effector_state

    @property
    def gripper_pos(self):
        """实际夹爪位置（单位：m）"""
        if self.end_effector_type != "gripper":
            return 0.0
        pos = self.end_effector_pos
        return pos[0] if pos else 0.0

    @property
    def gripper_tau(self):
        """实际夹爪力矩（单位：N）"""
        if self.end_effector_type != "gripper":
            return 0.0
        tau = self.end_effector_tau
        return tau[0] if tau else 0.0

    @property
    def plan_gripper_pos(self):
        """规划夹爪位置（单位：m）"""
        if self.end_effector_type != "gripper":
            return 0.0
        pos = self.plan_end_effector_pos
        return pos[0] if pos else 0.0

    @property
    def plan_gripper_tau(self):
        """规划夹爪力矩（单位：N）"""
        if self.end_effector_type != "gripper":
            return 0.0
        tau = self.plan_end_effector_tau
        return tau[0] if tau else 0.0

    @property
    def hand_state(self):
        """灵巧手状态（-1 未连接/无灵巧手，0 未使能，1 正常）"""
        eeff = self._arm_state.get("eeff", {})
        if not eeff.get("is_connect", False) or self.end_effector_type != "hand":
            return -1
        return self.end_effector_state

    @property
    def hand_pos(self):
        """实际灵巧手位置（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.end_effector_pos

    @property
    def hand_vel(self):
        """实际灵巧手速度（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.end_effector_vel

    @property
    def hand_tau(self):
        """实际灵巧手力矩（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.end_effector_tau

    @property
    def plan_hand_pos(self):
        """规划灵巧手位置（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.plan_end_effector_pos

    @property
    def plan_hand_vel(self):
        """规划灵巧手速度（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.plan_end_effector_vel

    @property
    def plan_hand_tau(self):
        """规划灵巧手力矩（列表）"""
        if self.end_effector_type != "hand":
            return []
        return self.plan_end_effector_tau

    # -------------------- 控制命令 --------------------
    def set_ready(self, timeout_ms=3000) -> bool:
        """
        将机械臂设置为就绪状态（错误清除、伺服上使能、切换到位置模式）。

        :param timeout_ms: int, 超时时间（毫秒），默认 3000ms

        :return: bool, True 表示就绪成功，False 表示失败
        """
        # 初始等待 0.1 秒，对应 C++ 的 usleep(100000)
        time.sleep(0.1)
        print("Setting arm ready...")

        start_time = time.time()
        deadline = start_time + timeout_ms / 1000.0

        # 确保至少收到一次状态，带超时
        while not self._arm_state:
            if time.time() > deadline:
                print("Error: Timeout waiting for initial arm state")
                return False
            time.sleep(0.1)

        if not self._running or not self.is_connected():
            print("Error: Disconnected or interrupted while waiting for initial arm state")
            return False
        
        arm = self._arm_state
        servo = arm.get("servo", 0)
        fsm_state = arm.get("fsm_state", "")
        state_val = arm.get("state", 0)  # -1 错误，0 空闲，1 运行，2 拖动

        # 检查是否已经就绪
        if (servo == 1 and fsm_state == "POSITION" and state_val != -1 and self.is_connected()):
            # 已就绪，执行清理和恢复（与 C++ 一致）
            self.clean_carm_error()
            self.__abort_all_tasks()  # 连接异常时利用安全中断机制，防止死锁
            self.recover()
            self.limit = self.get_limits()  # 更新配置
            self.eeff_limit = self.get_eeff_config()  # 更新末端配置
            return True

        # 辅助函数：检查是否满足就绪条件
        def is_ready():
            arm = self._arm_state
            if not arm:
                return False
            return (arm.get("servo", 0) == 1 and
                    arm.get("fsm_state", "") == "POSITION" and
                    arm.get("state", 0) != -1 and
                    self.is_connected())
        # 带超时等待
        if 'start_time' not in locals():
            start_time = time.time()
            deadline = start_time + timeout_ms / 1000.0

        self.clean_carm_error()
        # 第一步：清除错误
        if state_val == -1:
            self.clean_carm_error()
            while time.time() < deadline:
                time.sleep(0.1)
                # 重新获取状态
                if self._arm_state.get("state", 0) != -1:
                    break
                self.clean_carm_error()

        # 第二步：伺服使能
        if not servo:
            self.set_servo_enable(True)
            while time.time() < deadline:
                time.sleep(0.1)
                if self._arm_state.get("servo", 0) == 1:
                    break
                self.set_servo_enable(True)

        # 第三步：切换到位置模式
        if fsm_state != "POSITION":
            # C++ 中此处调用 set_control_mode(0) 可能是笔误，实际应设为位置模式 (1)
            self.set_control_mode(1)
            while time.time() < deadline:
                time.sleep(0.1)
                if self._arm_state.get("fsm_state", "") == "POSITION":
                    break
                self.set_control_mode(1)

        # 最终检查
        arm = self._arm_state
        print(f"Final arm state: servo={arm.get('servo', 0)}, fsm_state={arm.get('fsm_state', '')}, state={arm.get('state', 0)}")
        if is_ready():
            self.clean_carm_error()
            self.__abort_all_tasks()  # 利用安全中断机制防止死锁
            self.recover()
            self.limit = self.get_limits()
            self.eeff_limit = self.get_eeff_config()
            return True
        else:
            return False

    def ping(self, date=None) -> dict:
        """
        发送ping请求，底层返回收到消息的Unix时间戳，用于测试通讯延时以及带宽。
        由于该方法有多个参数用于网络和延迟分析，保留完整的字典返回。
        解析方式：
        - `res.get("Unix_resp")`: 请求发出时的时间戳，接收后被改成接收到的时间戳（单位：秒，字符串格式），两者的差值即为单程网络延时。
        - `res.get("data")`: 请求携带的数据载荷（date 参数）

        :param date: str, 携带的数据载荷

        :return: dict, 请求的完整响应字典
        """
        import time
        return self.request({
            "command": "ping",
            "Unix_resp": str(time.time()),
            "data": date
        })

    def set_servo_enable(self, enable=True) -> bool:
        """
        设置伺服使能

        :param enable: bool, True 使能，False 下使能
        """
        res = self.request({
            "command": "setServoEnable",
            "arm_index": self.arm_index,
            "enable": enable
        })
        return res.get("recv") == "Task_Recieve"

    def set_control_mode(self, mode=1) -> bool:
        """
        设置控制模式

        :param mode: int, 0-IDLE, 1-点位, 2-MIT, 3-拖动, 4-力位混合
        """
        mode = self.__clip(mode, 0, 4)
        res = self.request({
            "command": "setControlMode",
            "arm_index": self.arm_index,
            "mode": mode
        })
        return res.get("recv") == "Task_Recieve"

    # def set_impedance_mode(self, mode=0) -> bool:
    #     """
    #     设置控制器阻抗控制模式

    #     :param mode: int, 0-Joint impedance 关节阻抗模式, 1-Cartesian impedance 笛卡尔阻抗模式

    #     :return: bool, 执行是否成功
    #     """
    #     mode = self.__clip(mode, 0, 1)
    #     res = self.request({
    #         "command": "setImpedanceMode",
    #         "arm_index": self.arm_index,
    #         "mode": mode
    #     })
    #     return res.get("recv") == "Task_Recieve"

    # def set_teleoperation_mode(self, mode=0) -> bool:
    #     """
    #     设置遥操作模式

    #     :param mode: int, 主臂设为1, 从臂设为2, 退出0

    #     :return: bool, 执行是否成功
    #     """
    #     res = self.request({
    #         "command": "setTeleoperationMode",
    #         "arm_index": self.arm_index,
    #         "is_master": mode
    #     })
    #     return res.get("recv") == "Task_Recieve"

    def set_passthrough_data(self, mode, can_id, data) -> tuple:
        """
        设置透传数据

        :param mode: int, 模式，0-仅发送，1-仅接收，2-发送并接收

        :param can_id: int, CAN ID

        :param data: list/str, 透传数据（字节列表或十六进制字符串等，需底层支持）

        :return: tuple, 当模式为 1 或 2，并且执行成功时，直接返回两参数元组：(can_id, response_data_bytes)。
                 其它情况返回 bool 表示执行是否成功。
        """
        if isinstance(data, list):
            import binascii
            data = bytes(data).hex()

        res = self.request({
            "command": "setPassthroughData",
            "arm_index": self.arm_index,
            "mode": mode,
            "data": {
                "can_id": can_id,
                "data": data
            }
        })
        if res.get("recv") == "Task_Recieve":
            if res.get("ret", 0) == 1 and (mode == 1 or mode == 2):
                ret_data = res.get("data", {})
                return True, ret_data.get("can_id", can_id), bytes.fromhex(ret_data.get("data", ""))
            else:
                return res.get("ret", 0) == 1, None, None
        else:
            return False , None, None

    def set_ecat_passthrough_data(self, mode, frame, timeout_ms=100) -> tuple:
        """通过 EtherCAT 透传板同步发送或接收完整 CAN/CAN FD 帧。

        返回 ``(code, response_frame)``。code 含义：1 成功；-1 SDK 通信失败；
        -2 后端拒绝或执行失败；-3 后端响应格式异常；-4 Python 参数序列化失败。
        """
        if not isinstance(frame, dict):
            _LOGGER.error("set_ecat_passthrough_data: frame 必须是 dict")
            return -4, None

        sends_frame = mode in (0, 2)
        request_frame = {"can_fd": frame.get("can_fd", False)}
        if sends_frame:
            if "can_id" in frame:
                request_frame["can_id"] = frame["can_id"]
            if "flags" in frame:
                request_frame["flags"] = frame["flags"]
            if "data" in frame:
                data = frame["data"]
                try:
                    request_frame["data"] = data if isinstance(data, str) else bytes(data).hex()
                except (TypeError, ValueError):
                    _LOGGER.error("set_ecat_passthrough_data: data 无法转换为字节数据")
                    return -4, None

        request_data = {
            "command": "setEcatPassthroughData",
            "arm_index": self.arm_index,
            "mode": mode,
            "data": request_frame,
        }
        if mode == 2:
            request_data["timeout_ms"] = timeout_ms

        try:
            json.dumps(request_data)
        except (TypeError, ValueError):
            _LOGGER.error("set_ecat_passthrough_data: 请求参数无法序列化为 JSON")
            return -4, None

        request_timeout = 6.0 if mode == 2 else 1.0
        res = self.request(request_data, timeout=request_timeout)
        recv = res.get("recv")
        if recv != "Task_Recieve":
            error_msg = res.get("error_msg") or res.get("errMsg") or "后端拒绝请求"
            _LOGGER.error("set_ecat_passthrough_data: %s", error_msg)
            if "errMsg" in res and "error_msg" not in res:
                return -1, None
            return -2, None
        if type(res.get("ret")) is not bool:
            _LOGGER.error("set_ecat_passthrough_data: 后端响应缺少有效的 ret 字段")
            return -3, None
        if not res["ret"]:
            error_msg = res.get("error_msg") or "后端透传操作失败"
            _LOGGER.error("set_ecat_passthrough_data: %s", error_msg)
            return -2, None

        response = res.get("data")
        if not isinstance(response, dict):
            _LOGGER.error("set_ecat_passthrough_data: 后端响应缺少有效的 data 字段")
            return -3, None
        response_can_id = response.get("can_id")
        response_flags = response.get("flags")
        response_can_fd = response.get("can_fd")
        response_length = response.get("length")
        response_data = response.get("data")
        if (type(response_can_id) is not int or response_can_id < 0 or
                type(response_flags) is not int or not 0 <= response_flags <= 0xFF or
                type(response_can_fd) is not bool or type(response_length) is not int or
                response_length < 0 or not isinstance(response_data, str)):
            _LOGGER.error("set_ecat_passthrough_data: 后端返回帧字段格式异常")
            return -3, None

        try:
            if len(response_data) % 2 != 0:
                raise ValueError("十六进制字符串长度必须为偶数")
            payload = bytes.fromhex(response_data)
        except ValueError:
            _LOGGER.error("set_ecat_passthrough_data: 后端返回的 data 不是有效十六进制数据")
            return -3, None
        if response_length != len(payload) or len(payload) > (64 if response_can_fd else 8):
            _LOGGER.error("set_ecat_passthrough_data: 后端返回帧长度异常")
            return -3, None

        allowed_response_flags = (
            ECAT_CAN_FLAG_EXTENDED | ECAT_CAN_FLAG_BRS | ECAT_CAN_FLAG_ESI
            if response_can_fd else ECAT_CAN_FLAG_EXTENDED | ECAT_CAN_FLAG_REMOTE
        )
        max_response_can_id = (
            0x1FFFFFFF if response_flags & ECAT_CAN_FLAG_EXTENDED else 0x7FF
        )
        if response_flags & ~allowed_response_flags or response_can_id > max_response_can_id:
            _LOGGER.error("set_ecat_passthrough_data: 后端返回帧 flags 或 can_id 异常")
            return -3, None

        return 1, {
            "can_id": response_can_id,
            "flags": response_flags,
            "can_fd": response_can_fd,
            "data": list(payload),
        }

    def set_end_effector(self, dof, pos, vel, tau, control_motor=False) -> bool:
        """
        设置末端执行器（夹爪）的位置、速度、力矩，指定自由度 dof。
        输入可以是单个数值或列表，长度不足 dof 则补零，超出则截断。

        :param dof: int, 末端执行器自由度（整数）

        :param pos: float/list, 位置值或列表（单位：m or rad）

        :param vel: float/list, 速度值或列表（单位：m/s or rad/s）

        :param tau: float/list, 力矩值或列表（单位：N）

        :param control_motor: bool, ``True`` 直接控制末端电机，默认 ``False``。

        :return: bool, 执行是否成功
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel) or not self.__check_input_valid(tau):
            print(f"Error: set_end_effector input contains NaN or Inf")
            return False

        self.__clip_eeff(dof, pos, vel, tau, control_motor)
        res = self.request({
            "command": "setEffectorCtr",
            "arm_index": self.arm_index,
            "pos": pos,
            "vel": vel,
            "tau": tau,
            "control_motor": control_motor
        })
        return res.get("recv") == "Task_Recieve"

    def set_eeff(self, pos, vel, tau, control_motor=False) -> bool:
        """
        统一末端执行器控制接口，设置末端执行器的位置、速度、力矩。
        自动推断自由度，pos/vel/tau 维度需一致。

        :param pos: list, 位置值列表（单位：m or rad）

        :param vel: list, 速度值列表（单位：m/s or rad/s）

        :param tau: list, 力矩值列表（单位：N）

        :param control_motor: bool, ``True`` 直接控制末端电机，默认 ``False``。

        :return: bool, 执行是否成功
        """
        dof = max(len(pos) if isinstance(pos, list) else 0,
                  len(vel) if isinstance(vel, list) else 0,
                  len(tau) if isinstance(tau, list) else 0)
        if dof == 0 or len(tau) != dof or len(vel) != dof:
            print(f"Error: set_eeff dof={dof}, pos/vel/tau 维度不一致")
            return False
        return self.set_end_effector(dof, pos, vel, tau, control_motor)

    def set_gripper(self, pos, tau=10) -> bool:
        """设置夹爪位置和力矩（pos单位：m，tau单位：N）"""
        return self.set_eeff([pos], [0.0], [tau])

    def set_hand(self, pos, tau, vel) -> bool:
        """
        设置灵巧手位置、力矩和速度

        :param pos: float/list, 灵巧手位置或列表

        :param tau: float/list, 灵巧手力矩或列表

        :param vel: float/list, 灵巧手速度或列表

        :return: bool, 执行是否成功
        """
        return self.set_eeff(pos, vel, tau)

    def set_tool_index(self, index) -> bool:
        """
        设置当前工具号。

        :param index: int, 工具索引（整数，通常从 0 开始）

        :return: bool, 执行是否成功，可检查 recv 字段是否为 "Task_Recieve"
        """
        if not self.__check_input_valid(index):
            return False
        res = self.request({
            "command": "setToolData",
            "operation": "change",
            "index": index,
            "arm_index": self.arm_index
        })
        return res.get("recv") == "Task_Recieve"

    @property
    def tool_index(self):
        """
        获取当前工具号。

        :return: int, 当前工具索引（整数），若状态未更新或无此字段返回 0
        """
        return self._arm_state.get("tool", 0)
        
    def get_tool_coordinate(self, tool) -> list:
        """
        获取指定工具的坐标系（工具末端相对法兰的位姿）

        :param tool: int, 工具索引

        :return: list, 工具的笛卡尔坐标 [x, y, z, qx, qy, qz, qw]，执行失败或异常时返回空列表
        """
        if not self.__check_input_valid(tool):
            return []
        res = self.request({
            "command": "getCoordinate",
            "arm_index": self.arm_index,
            "type": "tool",
            "index": tool
        })
        if res.get("recv") == "Task_Recieve":
            return res.get("coordinate", [])
        return []

    # def set_tool_coordinate(self, index, coord) -> bool:
    #     """
    #     更新工具坐标系（工具末端相对法兰的位姿关系）

    #     :param index: int, 工具号索引

    #     :param coord: list, 笛卡尔坐标及姿态 [x, y, z, rx, ry, rz]，长度6

    #     :return: bool, 执行是否成功
    #     """
    #     if not self.__check_input_valid(coord):
    #         return False
    #     res = self.request({
    #         "command": "setToolData",
    #         "operation": "update",
    #         "index": index,
    #         "arm_index": self.arm_index,
    #         "data": {"coord": list(coord[:6])}
    #     })
    #     return res.get("recv") == "Task_Recieve"

    # def tool_calibtate(self, tool_index, pos) -> bool:
    #     """
    #     标定工具号

    #     :param tool_index: int, 标定的工具号

    #     :param pos: list, 标定位姿列表，每个元素为 [x, y, z, rx, ry, rz]；位置标定需至少4个点位，姿态标定需至少3个点位

    #     :return: bool, 标定是否成功
    #     """
    #     if not pos:
    #         return False
    #     for p in pos:
    #         if not self.__check_input_valid(p):
    #             return False
    #     res = self.request({
    #         "command": "toolCalibrate",
    #         "arm_index": self.arm_index,
    #         "data": {
    #             "tool_index": tool_index,
    #             "pos": [list(p[:6]) for p in pos]
    #         }
    #     })
    #     if res.get("recv") == "Task_Recieve":
    #         return res.get("result") == "ok"
    #     return False

    def set_collision_config(self, flag=True, level=0) -> bool:
        """
        设置碰撞检测

        :param flag: bool, 开关

        :param level: int, 灵敏度等级 0~2
        """
        level = self.__clip(level, 0, 2)
        res = self.request({
            "command": "setCollisionConfig",
            "arm_index": self.arm_index,
            "flag": flag,
            "level": level
        })
        return res.get("recv") == "Task_Recieve"

    def stop(self, type=0) -> bool:
        """
        停止机械臂

        :param type: int, 0-暂停, 1-停止, 2-禁用, 3-紧急停止
        """
        type = self.__clip(type, 0, 3)
        stop_id = ["SIG_ARM_PAUSE", "SIG_ARM_STOP", "SIG_ARM_DISABLE", "SIG_EMERGENCY_STOP"]
        res = self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": stop_id[type],
            "step_cnt": 5
        })
        return res.get("recv") == "Task_Recieve"

    def stop_task(self, at_once=False) -> bool:
        """
        停止当前任务

        :param at_once: bool, True 立即停止，False 完成当前任务后停止
        """
        res = self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": "SIG_TASK_STOP",
            "stop_at_once": at_once
        })
        return res.get("recv") == "Task_Recieve"

    def recover(self) -> bool:
        """恢复机械臂（退出暂停/急停）"""
        res = self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": "SIG_ARM_RECOVER",
            "step_cnt": 5
        })
        return res.get("recv") == "Task_Recieve"

    def clean_carm_error(self) -> bool:
        """清除控制器错误"""
        res = self.request({
            "command": "setControllerErrorReset",
            "arm_index": self.arm_index
        })
        return res.get("recv") == "Task_Recieve"

    def set_speed_level(self, level=5.0, response_level=20) -> bool:
        """
        设置速度等级

        :param level: float, 速度等级 0~10

        :param response_level: int, 过渡周期数 1~10000
        """
        if not self.__check_input_valid(level) or not self.__check_input_valid(response_level):
            return False
        level = self.__clip(level, 0, 10)
        response_level = self.__clip(response_level, 1, 10000)
        res = self.request({
            "command": "setSpeedLevel",
            "arm_index": self.arm_index,
            "level": level,
            "response_level": response_level
        })
        return res.get("recv") == "Task_Recieve"

    def set_debug(self, flag=False) -> bool:
        """
        设置控制器进入debug仿真模式，该模式下不连接机械臂

        :param flag: bool, True 开启仿真模式，False 关闭

        :return: bool, 执行是否成功
        """
        res = self.request({
            "command": "setDebugMode",
            "trigger": flag
        })
        return res.get("recv") == "Task_Recieve"

    def set_drag_params(self, torque_factor, friction_compensation_factor) -> bool:
        """
        设置拖动模式参数（转矩因子和摩擦力补偿系数）

        :param torque_factor: list, 转矩因子列表，范围 [0, 2]

        :param friction_compensation_factor: list, 摩擦力补偿系数列表，范围 [0, 2]

        :return: bool, 执行是否成功
        """
        if not self.__check_input_valid(torque_factor) or not self.__check_input_valid(friction_compensation_factor):
            return False
        res = self.request({
            "command": "setDragModeParams",
            "arm_index": self.arm_index,
            "params": {
                "torque_factor": list(torque_factor),
                "fric_compensation_factor": list(friction_compensation_factor)
            }
        })
        return res.get("recv") == "Task_Recieve"

    # -------------------- 运动接口 --------------------
    def track_joint(self, pos, end_effector=None) -> bool:
        """
        关节空间轨迹跟踪（周期性发送目标关节位置）

        :param pos: list, 目标关节位置

        :param end_effector: float, 夹爪位置（单位：m）

        :param tau: float, 夹爪力矩（单位：N）

        :return: bool, 是否成功发送
        """
        if not self.__check_input_valid(pos):
            return False
        if not self.__clip_joints(pos):
            return False
        req = {
            "command": "trajectoryTrackingTasks",
            "task_id": "TASK_TRACKING",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"way_point": pos}
        }
        if end_effector is not None:
            end_effector = self.__clip(end_effector, 0, 0.08)
            req["data"]["grp_point"] = end_effector
            
        return self.send_only(req)

    def track_pose(self, pos, end_effector=None) -> bool:
        """
        笛卡尔空间轨迹跟踪（周期性发送目标位姿）

        :param pos: list, 目标笛卡尔位姿 [x, y, z, qx, qy, qz, qw]

        :param end_effector: float, 夹爪位置（单位：m）

        :param tau: float, 夹爪力矩（单位：N）

        :return: bool, 是否成功发送
        """
        _pos = list(pos)
        if not self.__check_input_valid(_pos):
            return False
        
        # Normalize quaternion for track_pose
        if isinstance(_pos, list) and len(_pos) >= 7:
            norm = sum(x*x for x in _pos[3:7])
            if abs(norm - 1.0) > 1e-4 and norm > 0:
                scale = 1.0 / math.sqrt(norm)
                for i in range(3, 7):
                    _pos[i] *= scale
        req = {
            "command": "trajectoryTrackingTasks",
            "task_id": "TASK_TRACKING",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"way_point": _pos}
        }        
        if end_effector is not None:
            end_effector = self.__clip(end_effector, 0, 0.08)
            req["data"]["grp_point"] = end_effector

        return self.send_only(req)

    # def set_redundancy_tau(self, redundancy_tau, gripper=0) -> bool:
    #     """
    #     遥操作发送冗余力矩

    #     :param redundancy_tau: list, 冗余力矩列表

    #     :param gripper: float, 夹爪力矩

    #     :return: bool, 是否成功发送
    #     """
    #     if not self.__check_input_valid(redundancy_tau):
    #         return False
    #     if not self.__check_input_valid(gripper):
    #         return False
    #     return self.send_only({
    #         "command": "setRedundancyTau",
    #         "arm_index": self.arm_index,
    #         "is_master": True,
    #         "redundancy_tau": list(redundancy_tau),
    #         "eeff_tau": gripper
    #     })

    def move_joint(self, pos, desire_time: float=-1, is_sync=True, tool=0) -> bool:
        """
        关节点的关节空间点到点运动

        :param pos: list, 目标关节位置

        :param desire_time: float, 期望运动时间（秒），<0 时使用默认速度规划，>=0 时使用 PVT 模式在指定时间内到达

        :param is_sync: bool, 是否同步等待

        :param tool: int, 工具号
        """
        if not self.__check_input_valid(pos):
            return False
        if not self.__clip_joints(pos):
            return False
        if desire_time >= 0:
            return self.move_pvt([list(pos)], gripper_pos=None, stamps=[desire_time],
                                 is_joint_val=True, is_sync=is_sync)
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"tool": tool, "target_pos": pos, "speed": 100}
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_pose(self, pos, desire_time: float=-1, is_sync=True, tool=0) -> bool:
        """
        笛卡尔点的关节空间点到点运动

        :param pos: list, 目标位姿 [x, y, z, qx, qy, qz, qw]

        :param desire_time: float, 期望运动时间（秒），<0 时使用默认速度规划，>=0 时使用 PVT 模式在指定时间内到达

        :param is_sync: bool, 是否同步等待

        :param tool: int, 工具号
        """
        if not self.__check_input_valid(pos):
            return False
        if not self.__check_normalized(pos):
            return False
        if desire_time >= 0:
            return self.move_pvt([list(pos)], gripper_pos=None, stamps=[desire_time],
                                 is_joint_val=False, is_sync=is_sync)
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"tool": tool, "target_pos": pos, "speed": 100}
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_line_pose(self, pos, is_sync=True, tool=0) -> bool:
        """
        笛卡尔点的空间直线运动

        :param pos: list, 目标位姿 [x, y, z, qx, qy, qz, qw]

        :param is_sync: bool, 是否同步等待

        :param tool: int, 工具号
        """
        if not self.__check_input_valid(pos):
            return False
        if not self.__check_normalized(pos):
            return False
        # 注意：这里传入的是笛卡尔位姿，point_type=1，直接发送位姿
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVL",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"tool": tool, "point": pos, "speed": 100}
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_line_joint(self, pos, is_sync=True, tool=0) -> bool:
        """
        关节点的空间直线运动

        :param pos: list, 目标关节位置

        :param is_sync: bool, 是否同步等待

        :param tool: int, 工具号
        """
        if not self.__check_input_valid(pos):
            return False
        if not self.__clip_joints(pos):
            return False
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVL",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"tool": tool, "point": pos, "speed": 100}
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_flow_pose(self, target_pos, line_theta_weight=0.5, accuracy=0.0001, move_line=False, is_sync=True, tool=0) -> bool:
        """
        笛卡尔雅可比迭代运动（TASK_FLOW）

        :param target_pos: list, 目标位姿 [x, y, z, qx, qy, qz, qw]

        :param line_theta_weight: float, 位置与姿态的权重（0~1）

        :param accuracy: float, 收敛精度

        :param move_line: bool, 是否直线运动

        :param is_sync: bool, 是否同步等待

        :param tool: int, 工具号
        """
        if not self.__check_input_valid(target_pos):
            return False
        if not self.__check_normalized(target_pos):
            return False
        line_theta_weight = self.__clip(line_theta_weight, 0, 1)
        accuracy = self.__clip(accuracy, 1e-6, 1.0)  # 最小精度限制，避免过小导致计算问题
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_FLOW",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "data": {
                "target_pos": target_pos,
                "speed": 100,          # 默认速度
                "acc": 100,
                "tool": tool
            },
            "params": {
                "accuracy": accuracy,
                "line_theta_weight": line_theta_weight,
                "use_line": move_line
            }
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_toppra(self, targets, speed=100, tool=0, is_joint_val=True, is_sync=True) -> bool:
        """
        基于 TOPPRA 的多点轨迹运动

        :param targets: list, 目标轨迹点列表，关节位置列表或位姿列表 [x, y, z, qx, qy, qz, qw]

        :param speed: int, 速度百分比

        :param tool: int, 工具号

        :param is_joint_val: bool, True 表示关节空间目标，False 表示笛卡尔空间目标

        :param is_sync: bool, 是否同步等待
        """
        if not targets:
            return False
        
        if not isinstance(targets[0], list):
            targets = [targets]
        
        for p in targets:
            if not self.__check_input_valid(p):
                return False
            if is_joint_val:
                if not self.__clip_joints(p):
                    return False
            else:
                if not self.__check_normalized(p):
                    return False
        
        req = {
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVT",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0 if is_joint_val else 1},
            "data": {
                "point_num": len(targets),
                "speed": speed,
                "acc": speed,
                "tool": tool
            }
        }
        
        for i, target in enumerate(targets):
            req["data"][f"way_point{i+1}"] = list(target)
            
        res = self.request(req)
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_pvt(self, target_pos, gripper_pos=None, stamps=None, is_joint_val=True, is_sync=True) -> bool:
        """
        PVT（位置-速度-时间）模式运动，无需加入起始点，直接给目标点

        :param target_pos: list, 规划路点列表，每个路点为关节角或位姿（n × DOF 或 n × 7）

        :param gripper_pos: list, 夹爪位置列表，与路点一一对应，None 或空为不运动夹爪

        :param stamps: list, 时间戳列表（秒），与路点一一对应，必须递增且大于0

        :param is_joint_val: bool, True 表示关节空间目标，False 表示笛卡尔空间目标

        :param is_sync: bool, 是否同步等待任务完成

        :return: bool, 执行是否成功
        """
        if not target_pos or not stamps:
            return False
        if len(target_pos) != len(stamps):
            print("Error: move_pvt target_pos and stamps length mismatch")
            return False
        for p in target_pos:
            if not self.__check_input_valid(p):
                return False
        for s in stamps:
            if not self.__check_input_valid(s) or s <= 0:
                print("Error: move_pvt stamps must be positive and increasing")
                return False
        req = {
            "command": "webRecieveTasks",
            "task_id": "TASK_PVT",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0 if is_joint_val else 1},
            "data": {
                "pos": [list(p) for p in target_pos],
                "time": list(stamps)
            },
            "gripper": {"eeff_linkage": 0}
        }
        if gripper_pos:
            if len(gripper_pos) != len(target_pos):
                print("Error: move_pvt gripper_pos length mismatch")
                return False
            req["gripper"]["eeff_linkage"] = 1
            req["gripper"]["eeffe_point"] = [max(0.0, min(0.08, g)) for g in gripper_pos]
        res = self.request(req)
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def move_joint_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True) -> bool:
        """
        关节轨迹运动，stamps 为空时使用 TOPPA 路径规划，有值时使用 PVT 模式

        :param target_traj: list, 目标关节位置轨迹列表

        :param gripper_pos: list, 夹爪位置列表（仅 PVT 模式生效）

        :param stamps: list, 时间戳列表（秒），None 或空时使用 TOPPA，有值时使用 PVT

        :param is_sync: bool, 是否同步等待
        """
        if stamps:
            return self.move_pvt(target_traj, gripper_pos=gripper_pos, stamps=stamps,
                                 is_joint_val=True, is_sync=is_sync)
        return self.move_toppra(target_traj, is_joint_val=True, is_sync=is_sync)

    def move_pose_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True) -> bool:
        """
        位姿轨迹运动，stamps 为空时使用 TOPPA 路径规划，有值时使用 PVT 模式

        :param target_traj: list, 目标笛卡尔位姿轨迹列表

        :param gripper_pos: list, 夹爪位置列表（仅 PVT 模式生效）

        :param stamps: list, 时间戳列表（秒），None 或空时使用 TOPPA，有值时使用 PVT

        :param is_sync: bool, 是否同步等待
        """
        if stamps:
            return self.move_pvt(target_traj, gripper_pos=gripper_pos, stamps=stamps,
                                 is_joint_val=False, is_sync=is_sync)
        return self.move_toppra(target_traj, is_joint_val=False, is_sync=is_sync)

    # def move_joint_jog(self, joint_index, dir, speed) -> bool:
    #     """
    #     单轴点动，运动至限位停止运动，可配合 stop 接口提前停止

    #     :param joint_index: int, 关节轴号，从0开始

    #     :param dir: int, 方向，1-正向，-1-反向

    #     :param speed: float, 速度百分比，（0，100]

    #     :return: bool, 执行是否成功
    #     """
    #     res = self.request({
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_MOVJ_JOG",
    #         "arm_index": self.arm_index,
    #         "index": joint_index,
    #         "dir": dir,
    #         "data": {"speed": speed}
    #     })
    #     return res.get("recv") == "Task_Recieve"

    # def move_line_jog(self, space_index, dir, speed, tool=0) -> bool:
    #     """
    #     笛卡尔空间单轴点动

    #     :param space_index: int, 空间维度标号，X:0，Y:1，Z:2，RX:3，RY:4，RZ:5

    #     :param dir: int, 方向，1-正向，-1-反向

    #     :param speed: float, 速度百分比，（0，100]

    #     :param tool: int, 工具号，0-9

    #     :return: bool, 执行是否成功
    #     """
    #     res = self.request({
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_MOVL_JOG",
    #         "arm_index": self.arm_index,
    #         "index": space_index,
    #         "dir": dir,
    #         "data": {"speed": speed, "acc": speed, "tool": tool}
    #     })
    #     return res.get("recv") == "Task_Recieve"

    # def move_3p_arc(self, arc_pos, speed=100, dir=True, angle=-1, tool=0, is_joint_val=True, is_sync=True) -> bool:
    #     """
    #     三点圆弧运动

    #     :param arc_pos: list, 包含3个点的列表（关节位置或位姿）

    #     :param speed: float, 速度百分比

    #     :param dir: bool, 圆弧方向，True-正向，False-反向

    #     :param angle: float, 圆弧角度，-1表示自动

    #     :param tool: int, 工具号

    #     :param is_joint_val: bool, 目标点是否为关节空间点

    #     :param is_sync: bool, 是否同步等待

    #     :return: bool, 执行是否成功
    #     """
    #     if len(arc_pos) != 3:
    #         print("Error: move_3p_arc requires exactly 3 points")
    #         return False
    #     for p in arc_pos:
    #         if not self.__check_input_valid(p):
    #             return False
    #         if is_joint_val:
    #             if not self.__clip_joints(p):
    #                 return False
    #         else:
    #             if not self.__check_normalized(p):
    #                 return False
    #     data = {
    #         "speed": speed,
    #         "acc": speed,
    #         "type": 0,
    #         "dir": 0 if dir else 1,
    #         "tool": tool,
    #         "points": [list(p) for p in arc_pos]
    #     }
    #     if angle > 0:
    #         data["radian"] = angle
    #     res = self.request({
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_MOVC",
    #         "task_level": "Task_General",
    #         "arm_index": self.arm_index,
    #         "point_type": {"space": 0 if is_joint_val else 1},
    #         "data": data
    #     })
    #     if is_sync and res.get("recv") == "Task_Recieve":
    #         self.__wait_task(res.get("task_key"))
    #     return res.get("recv") == "Task_Recieve"

    # def move_center_arc(self, arc_pos, speed=100, dir=True, angle=-1, tool=0, is_joint_val=True, is_sync=True) -> bool:
    #     """
    #     圆心圆弧运动（圆心、经过点、目标点）

    #     :param arc_pos: list, 包含3个点的列表（圆心、经过点、目标点）

    #     :param speed: float, 速度百分比

    #     :param dir: bool, 圆弧方向，True-正向，False-反向

    #     :param angle: float, 圆弧角度，-1表示自动

    #     :param tool: int, 工具号

    #     :param is_joint_val: bool, 目标点是否为关节空间点

    #     :param is_sync: bool, 是否同步等待

    #     :return: bool, 执行是否成功
    #     """
    #     if len(arc_pos) != 3:
    #         print("Error: move_center_arc requires exactly 3 points")
    #         return False
    #     for p in arc_pos:
    #         if not self.__check_input_valid(p):
    #             return False
    #         if is_joint_val:
    #             if not self.__clip_joints(p):
    #                 return False
    #         else:
    #             if not self.__check_normalized(p):
    #                 return False
    #     data = {
    #         "speed": speed,
    #         "acc": speed,
    #         "type": 1,
    #         "dir": 0 if dir else 1,
    #         "tool": tool,
    #         "points": [list(p) for p in arc_pos]
    #     }
    #     if angle > 0:
    #         data["radian"] = angle
    #     res = self.request({
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_MOVC",
    #         "task_level": "Task_General",
    #         "arm_index": self.arm_index,
    #         "point_type": {"space": 0 if is_joint_val else 1},
    #         "data": data
    #     })
    #     if is_sync and res.get("recv") == "Task_Recieve":
    #         self.__wait_task(res.get("task_key"))
    #     return res.get("recv") == "Task_Recieve"

    # def move_jump(self, target_cart_pos, start_jump_height, end_jump_height, speed=100, tool=0, is_sync=True) -> bool:
    #     """
    #     跳跃运动（门字形轨迹）

    #     :param target_cart_pos: list, 目标位姿 [x, y, z, qx, qy, qz, qw]

    #     :param start_jump_height: list, 起始点起跳高度偏移 [dx, dy, dz]

    #     :param end_jump_height: list, 目标点落点高度偏移 [dx, dy, dz]

    #     :param speed: float, 速度百分比

    #     :param tool: int, 工具号

    #     :param is_sync: bool, 是否同步等待

    #     :return: bool, 执行是否成功
    #     """
    #     if not self.__check_input_valid(target_cart_pos):
    #         return False
    #     if not self.__check_normalized(target_cart_pos):
    #         return False
    #     # 构造门字形路径点：当前位置 → 起跳 → 目标上方 → 目标
    #     # 先获取当前位姿作为起点
    #     current = self.cart_pose
    #     if not current or len(current) != 7:
    #         print("Error: move_jump cannot get current cart_pose")
    #         return False
    #     p0 = list(current)
    #     p1 = [p0[i] + start_jump_height[i] for i in range(3)] + p0[3:]
    #     p2 = list(target_cart_pos)
    #     p3 = [target_cart_pos[i] + end_jump_height[i] for i in range(3)] + list(target_cart_pos[3:])
    #     targets = [p0, p1, p3, p2]
    #     return self.move_toppra(targets, speed=speed, tool=tool, is_joint_val=False, is_sync=is_sync)

    # def move_directly(self, target_traj, gripper_pos=None, is_sync=True) -> bool:
    #     """
    #     直接运行已规划好的关节轨迹

    #     :param target_traj: list, 目标轨迹列表（二维列表，每行一个关节位置点）

    #     :param gripper_pos: list, 夹爪位置列表（可选，与轨迹点一一对应）

    #     :param is_sync: bool, 是否同步等待

    #     :return: bool, 执行是否成功
    #     """
    #     if not target_traj:
    #         return False
    #     for p in target_traj:
    #         if not self.__check_input_valid(p):
    #             return False
    #     req = {
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_MOVD",
    #         "task_level": "Task_General",
    #         "arm_index": self.arm_index,
    #         "target_points": [list(p) for p in target_traj],
    #         "gripper": {"eeff_linkage": 0}
    #     }
    #     if gripper_pos:
    #         req["gripper"]["eeff_linkage"] = 1
    #         req["gripper"]["eeffe_point"] = [[max(0.0, min(0.08, g))] for g in gripper_pos]
    #     res = self.request(req)
    #     if is_sync and res.get("recv") == "Task_Recieve":
    #         self.__wait_task(res.get("task_key"))
    #     return res.get("recv") == "Task_Recieve"

    # def move_delay(self, delay_ms, priority=0, is_sync=True) -> bool:
    #     """
    #     延迟任务，机器人保持非standby状态但静止

    #     :param delay_ms: float, 延迟时间（毫秒）

    #     :param priority: int, 优先级，0-一般，1-高

    #     :param is_sync: bool, 是否同步等待

    #     :return: bool, 执行是否成功
    #     """
    #     task_level = "Task_Override" if priority == 1 else "Task_General"
    #     res = self.request({
    #         "command": "webRecieveTasks",
    #         "task_id": "TASK_TIMER",
    #         "task_level": task_level,
    #         "arm_index": self.arm_index,
    #         "data": {"time": delay_ms}
    #     })
    #     if is_sync and res.get("recv") == "Task_Recieve":
    #         self.__wait_task(res.get("task_key"))
    #     return res.get("recv") == "Task_Recieve"

    # -------------------- 示教接口 --------------------
    def trajectory_teach(self, off_on, name="") -> bool:
        """
        开始/停止示教录制

        :param off_on: bool, True 开始录制，False 停止录制

        :param name: str, 轨迹名称（如 "20250918175001.my_traj"）
        """
        res = self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 1 if off_on else 0,
            "name": name
        })
        return res.get("recv") == "Task_Recieve"

    def trajectory_recorder(self, name, is_sync=True) -> bool:
        """
        复现指定名称的轨迹

        :param name: str, 轨迹名称

        :param is_sync: bool, 是否同步等待
        """
        res = self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 2,
            "name": name
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res.get("recv") == "Task_Recieve"

    def check_teach(self) -> list:
        """
        获取已录制的轨迹列表，只有正则匹配上的 '20250918175001.self_name.json' 才会被返回

        :return: list, 轨迹列表的字符串数组
        """
        res = self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 3,
            "name": "no_regular_expression"
        })
        if res.get("recv") == "Task_Recieve" and res.get("teach_list"):
            return res["teach_list"]
        return []

    # def set_traj_recorder(self, traj_record_flag) -> bool:
    #     """
    #     打开和关闭实际运行状态记录功能，包括位置和力矩，用于动力学参数辨识

    #     :param traj_record_flag: int, 记录模式：
    #         -1: 关闭轨迹记录
    #          0: 用于拖动示教，保存关节角
    #          1: 用于动力学参数辨识，保存关节角和力矩
    #          2: 用于检查动力学参数是否正确，保存计算力矩和反馈力矩
    #          3: 记录关节插补轨迹和反馈实际轨迹，用于验证轨迹跟随情况

    #     :return: bool, 执行是否成功
    #     """
    #     res = self.request({
    #         "command": "setTrajRecorder",
    #         "arm_index": self.arm_index,
    #         "toRecord": traj_record_flag
    #     })
    #     return res.get("recv") == "Task_Recieve"

    # -------------------- 运动学 --------------------
    def inverse_kine(self, cart_pose, ref_joints, tool=0) -> list:
        """
        逆运动学求解

        :param cart_pose: list, 目标位姿 [x, y, z, qx, qy, qz, qw]（单个或列表）

        :param ref_joints: list, 参考关节角（单个或列表）

        :param tool: int, 工具号

        :return: list, 成功时返回解析过的结果（单个一维列表或二维列表）逆解失败则返回该空列表，如果错误返回空列表
        """
        if not self.__check_input_valid(cart_pose) or not self.__check_input_valid(ref_joints):
            return []

        is_list = True
        if not isinstance(cart_pose[0], list):
            cart_pose = [cart_pose]
            ref_joints = [ref_joints]
            is_list = None
        assert len(cart_pose) == len(ref_joints)

        for p in cart_pose:
            if not self.__check_normalized(p):
                return []
        
        data = {"tool": tool, "point_cnt": len(cart_pose)}
        for i in range(len(ref_joints)):
            data[f"point{i+1}"] = cart_pose[i]
            data[f"refer{i+1}"] = ref_joints[i]
            
        ret = self.request({
            "command": "getKinematics",
            "task_id": "inverse",
            "arm_index": self.arm_index,
            "data": data
        })
        try:
            if ret.get("recv") == "Task_Recieve":
                if is_list:
                    joints = []
                    for i in range(len(cart_pose)):
                        joints.append(ret.get("data", {}).get(f"joint{i+1}", []))
                    return joints
                else:
                    return ret.get("data", {}).get("joint1", [])
            return []
        except Exception as e:
            print(f"Error parsing inverse_kine response: {e}")
            return []

    def forward_kine(self, joint_pos, tool=0) -> list:
        """
        正运动学求解

        :param joint_pos: list, 关节角（单个或列表）

        :param tool: int, 工具号

        :return: list, 成功时返回解析过的结果（单个一维列表或二维列表）逆解失败则返回该空列表[x, y, z, qx, qy, qz, qw]，如果错误返回空列表
        """
        if not self.__check_input_valid(joint_pos):
            print(f"Error: forward_kine input contains NaN or Inf")
            return []

        is_list = True
        if not isinstance(joint_pos[0], list):
            joint_pos = [joint_pos]
            is_list = False

        for v in joint_pos:
            if not self.__clip_joints(v):
                print(f"Error: forward_kine joint size error for {v}")
                return []

        data = {"tool": tool, "point_cnt": len(joint_pos)}
        for i in range(len(joint_pos)):
            data[f"joint{i+1}"] = joint_pos[i]
        ret = self.request({
            "command": "getKinematics",
            "task_id": "forward",
            "arm_index": self.arm_index,
            "data": data
        })
        try:
            if ret.get("recv") == "Task_Recieve":
                if is_list:
                    points = []
                    for i in range(len(joint_pos)):
                        points.append(ret.get("data", {}).get(f"point{i+1}", []))
                    return points
                else:
                    return ret.get("data", {}).get("point1", [])
            return []
        except Exception as e:
            print(f"Error parsing forward_kine response: {e}")
            return []

    # -------------------- 标定 / 自研臂接口 --------------------
    # def set_zero_position(self, axis_id) -> bool:
    #     """
    #     设置自研臂的零位（仅自研臂有效）

    #     :param axis_id: int, 关节轴号

    #     :return: bool, 控制器返回值，True 表示成功
    #     """
    #     res = self.request({
    #         "command": "setZeroPosition",
    #         "arm_index": self.arm_index,
    #         "axis_id": axis_id
    #     })
    #     if res.get("recv") == "Task_Recieve":
    #         return res.get("ret", -1) == 1
    #     return False

    # def check_dm_motor_id(self) -> list:
    #     """
    #     检查 DM 电机 ID（仅自研臂有效）

    #     :return: list, 电机 ID 列表，失败返回 [-1]
    #     """
    #     res = self.request({
    #         "command": "checkDmMotorId",
    #         "arm_index": self.arm_index
    #     })
    #     if res.get("recv") == "Task_Recieve":
    #         return res.get("ret", [-1])
    #     return [-1]

    # def set_dynamics_cali_record_flag(self) -> int:
    #     """
    #     动力学参数辨识数据记录

    #     :return: int, 控制器返回值，1 表示成功
    #     """
    #     res = self.request({
    #         "command": "dynamicsCalibration",
    #         "operation": "record"
    #     })
    #     if res.get("recv") == "Task_Recieve":
    #         return res.get("ret", -1) == 1
    #     return False

    # def set_dynamics_cali_calcu(self, is_payload=False, tool_index=0) -> int:
    #     """
    #     动力学参数辨识计算

    #     :param is_payload: bool, True 表示辨识负载参数，False 表示辨识臂参数

    #     :param tool_index: int, 工具号，辨识负载时工具号需大于5

    #     :return: int, 控制器返回值，1 表示成功
    #     """
    #     req = {
    #         "command": "dynamicsCalibration",
    #         "arm_index": self.arm_index,
    #         "operation": "calibrate",
    #         "type": "payload" if is_payload else "arm"
    #     }
    #     if is_payload:
    #         if tool_index < 5:
    #             print("Error: payload tool_index must be >= 5")
    #             return -1
    #         req["tool_index"] = tool_index
    #     res = self.request(req)
    #     if res.get("recv") == "Task_Recieve":
    #         return res.get("ret", -1) == 1
    #     return False

    # -------------------- 回调注册 --------------------
    def on_error(self, callback):
        """
        注册错误回调函数

        :param callback: callable, 函数签名 fn(error_info)
        """
        self.call_back["onCarmError"] = callback

    def on_task_finish(self, callback):
        """
        注册任务完成回调函数

        :param callback: callable, 函数签名 fn(task_key)
        """
        self.call_back["taskFinished"] = callback

    def on_update(self, callback):
        """
        注册状态更新回调函数

        :param callback: callable, 函数签名 fn(Unix_time)
        """
        self.call_back["updateRobotState"] = callback

    # -------------------- 请求/响应核心 --------------------
    def request(self, req, timeout=1):
        event = threading.Event()
        task_key = str(uuid.uuid4())
        req["task_key"] = task_key
        self.res_pool[task_key] = {"req": req, "event": event}
        
        if not self.__send(req):
            self.res_pool.pop(task_key, None)
            return {'recv': 'Task_Reject', 'errMsg': 'WebSocket not connected'}
            
        if not event.wait(timeout=timeout):
            self.res_pool.pop(task_key, None)
            return {'recv': 'Task_Reject', 'errMsg': 'Request timed out'}
            
        data = self.res_pool.pop(task_key, {})
        return data.get("res", {'recv': 'Task_Reject', 'errMsg': 'No response data'})

    def send_only(self, req):
        """仅发送请求，不等待响应"""
        task_key = str(uuid.uuid4())
        req["task_key"] = task_key
        return self.__send(req)

    def __send(self, msg):
        if self.ws and self.open_ready.is_set():
            try:
                self.ws.send(json.dumps(msg))
                return True
            except Exception as e:
                print(f"WebSocket send error: {e}")
                return False
        else:
            print("WebSocket not connected, cannot send message.")
            return False

    # -------------------- 回调处理 --------------------
    def __cbk_status(self, message):
        if "arm" not in message or not isinstance(message["arm"], list):
            return

        self.state = message

        # 触发状态更新回调
        if self._arm_state:
            self.call_back.get("updateRobotState", lambda msg: None)(self.state.get("Unix_time_stamp", 0))

        # 全局错误解析
        if message.get("error", 0) != 0 or message.get("errMsg", ""):
            error_arm_index = message.get("error_arm_index", -1)
            error_all_arm = message.get("error_all_arm", False)
            if error_arm_index == self.arm_index or error_all_arm:
                error_info = {
                    "command": "onCarmError",
                    "error": message.get("error"),
                    "errMsg": message.get("errMsg"),
                    "error_arm_index": error_arm_index
                }
                self.call_back.get("onCarmError", lambda msg: None)(error_info)
                if message.get("error") != 3001:
                    self.__abort_all_tasks()

        # 任务完成解析（只处理当前臂）
        if len(message["arm"]) > self.arm_index:
            arm_json = message["arm"][self.arm_index]
            if "task" in arm_json:
                task_info = arm_json["task"]
                if "last_task_key" in task_info and task_info["last_task_key"]:
                    task_key = task_info["last_task_key"]
                    self.call_back.get("taskFinished", lambda msg: None)(task_key)
                    # 触发特定的 task_pool 事件
                    if task_key in self.task_pool:
                        self.task_pool[task_key].set()
                            

    def __cbk_taskfinish(self, message):
        # print(f"Task finished callback received: {message}")
        pass

    def __cbk_error(self, message):
        print(f"Error callback received: {message}")

    def __cbk_update(self, message):
        pass

    def __on_open(self, ws):
        self.open_ready.set()
        self._retry_delay = 1.0  # 连接成功后重置重连延迟
        print("Connected successfully.")
        
        def fetch_limits():
            try:
                if self.is_connected():
                    res = self.get_limits()
                    if res:
                        self.limit = res
                    eeff_res = self.get_eeff_config()
                    if eeff_res:
                        self.eeff_limit = eeff_res
                else: 
                    print("Not connected, skipping limits fetch.")
            except Exception as e:
                print(f"Error fetching limits/eeff_limits: {e}")
                    
        threading.Thread(target=fetch_limits, daemon=True).start()

    def __on_close(self, ws, code, close_msg):
        print("Disconnected, please check your --addr", code, close_msg)
        self.open_ready.clear()
        self.__abort_all_tasks()  # 连接被动断开时，立即中断所有挂起的请求和等待，防止阻塞和不可预知的状态不同步

    def __on_message(self, ws, message):
        try:
            msg = json.loads(message)
            self.last_msg = msg
            cmd = msg.get("command")
            if not cmd:
                self.__response_op(msg)
                return
            op = self.ops.get(cmd, lambda msg: self.__response_op(msg))
            op(msg)
        except Exception as e:
            print(f"__on_message 异常: {e}, message: {message}")

    def __response_op(self, res):
        task_key = res.get("task_key", "")
        data = self.res_pool.get(task_key)
        if data:
            data["res"] = res
            data["event"].set()

    def __recv_loop(self):
        print("Recv loop started.")
        current_thread = threading.current_thread()
        while self._running and getattr(self, 'reader', None) is current_thread:
            local_ws = getattr(self, 'ws', None)
            if not local_ws:
                break
                
            try:
                local_ws.run_forever()
            except Exception as e:
                print(f"run_forever 异常: {e}")
            finally:
                # 只清理当前活动的连接状态
                if getattr(self, 'reader', None) is current_thread:
                    self.open_ready.clear()
                # 显式关闭并释放资源，防止底层的 sock 泄露与阻塞
                self.__force_close_ws_socket(local_ws)
                print("WebSocket connection closed, exiting recv loop iteration.")

            if self._running and getattr(self, 'reader', None) is current_thread:
                print(f"Connection lost. Reconnecting in {self._retry_delay} seconds...")
                self._reconnect_event.wait(self._retry_delay)
                self._reconnect_event.clear() # 重置为下一次 wait
                # 在等待期间可能调用了 close()，需要再次检查状态以决定是否继续重连
                if not self._running or getattr(self, 'reader', None) is not current_thread:
                    break
                    
                self._retry_delay = min(self._retry_delay * 1.5, self._max_delay)
                self._create_connection()
        print("Recv loop thread exiting.")

    def __wait_task(self, task_key, timeout=999999):
        # 使用独立的 task_key 分离事件，支持多线程并发等待
        event = threading.Event()
        self.task_pool[task_key] = event
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            # 每延时0.1秒检查一次连接情况，防止断联后的死锁
            if event.wait(timeout=0.1):
                break
            if not self._running or not self.is_connected():
                print("Warning: Connection lost or object destroyed while waiting for task.")
                break
                
        self.task_pool.pop(task_key, None)

    def __clip(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def __ensure_limit(self):
        """确保关节限位参数已获取，为 None 时自动获取一次"""
        if self.limit is None and self.is_connected():
            try:
                res = self.get_limits()
                if res:
                    self.limit = res
            except Exception as e:
                print(f"Error: auto-fetch limits failed: {e}")

    def __ensure_eeff_limit(self):
        """确保末端限位参数已获取，为 None 时自动获取一次"""
        if self.eeff_limit is None and self.is_connected():
            try:
                res = self.get_eeff_config()
                if res:
                    self.eeff_limit = res
            except Exception as e:
                print(f"Error: auto-fetch eeff_limit failed: {e}")

    def __clip_joints(self, joints):
        self.__ensure_limit()
        if not self.limit:
            return True

        lower = self.limit.get('limit_lower', [])
        upper = self.limit.get("limit_upper", [])
        
        if len(lower) != len(joints) or len(upper) != len(joints):
            return False

        for i, v in enumerate(joints):
            joints[i] = self.__clip(v, lower[i], upper[i])
        return True

    def __clip_joints_vel(self, joints_vel):
        self.__ensure_limit()
        if not self.limit:
            return True
            
        vmax = self.limit.get('joint_vel', [])

        if len(vmax) != len(joints_vel):
            return False

        for i, v in enumerate(joints_vel):
            joints_vel[i] = self.__clip(v, -vmax[i], vmax[i])
        return True
    
    def __clip_eeff(self, dof, eeff_pos, eeff_vel, eeff_tau, control_motor=False):
        def auto_pad(lst, target_len, default_val=0.0):
            if isinstance(lst, (int, float)):
                # 单个数值，转为列表，然后补零或截断
                lst = [float(lst)]
            else:
                lst = list(lst)

            if len(lst) < target_len:
                lst.extend([default_val] * (target_len - len(lst)))
            elif len(lst) > target_len:
                del lst[target_len:]
            return lst

        self.__ensure_eeff_limit()
        if not self.eeff_limit:
            return True

        if dof != self.eeff_limit.get("dof", 0):
            return False

        if control_motor:
            lower = self.eeff_limit.get("motor_lower", self.eeff_limit.get("eeff_lower", []))
            upper = self.eeff_limit.get("motor_upper", self.eeff_limit.get("eeff_upper", []))
            vel = self.eeff_limit.get("motor_vel", self.eeff_limit.get("eeff_vel", []))
            tau = self.eeff_limit.get("motor_tau", self.eeff_limit.get("eeff_tau", []))
        else:
            lower = self.eeff_limit.get("eeff_lower", [])
            upper = self.eeff_limit.get("eeff_upper", [])
            vel = self.eeff_limit.get("eeff_vel", [])
            tau = self.eeff_limit.get("eeff_tau", [])
        if not all(len(values) == dof for values in (lower, upper, vel, tau)):
            return False

        # 自动找齐
        eeff_pos[:] = auto_pad(eeff_pos, dof)
        eeff_tau[:] = auto_pad(eeff_tau, dof)
        eeff_vel[:] = auto_pad(eeff_vel, dof)
        # 自动限制
        for i, v in enumerate(eeff_pos):
            eeff_pos[i] = self.__clip(v, lower[i], upper[i])
        for i, v in enumerate(eeff_tau):
            eeff_tau[i] = self.__clip(v, -tau[i], tau[i])
        for i, v in enumerate(eeff_vel):
            eeff_vel[i] = self.__clip(v, -vel[i], vel[i])
        return True


    def __check_input_valid(self, values):
        """检查输入值是否包含 NaN 或 Inf"""
        if values is None or (isinstance(values, list) and len(values) == 0):
            print("Error: Input is None or empty list")
            return False
        if isinstance(values, (int, float)):
            if math.isnan(values) or math.isinf(values):
                print(f"Error: Input contains NaN or Inf: {values}")
                return False
            return True
        elif isinstance(values, list):
            for v in values:
                if not self.__check_input_valid(v):
                    return False
        return True

    def __check_normalized(self, pose):
        """Check if quaternion in pose (x, y, z, qx, qy, qz, qw) is normalized"""
        if isinstance(pose, list) and len(pose) >= 7:
             # Quaternion at index 3,4,5,6
             norm = sum(x*x for x in pose[3:7])
             if abs(norm - 1.0) > 1e-4:
                 print(f"Error: Quaternion not normalized (norm={norm})")
                 return False
        return True

    def __abort_all_tasks(self):
        """内部方法：安全中断所有正在阻塞等待的挂起任务和请求响应。
        防止调用 clear 后正在等待的线程永远阻塞死锁。
        """
        for event in list(self.task_pool.values()):  # list() 避免迭代时受其他线程删除元素导致异常
            event.set()
        self.task_pool.clear()
        
        for data in list(self.res_pool.values()):  # list 避免迭代中字典大小改变
            data["event"].set()
        self.res_pool.clear()

    def __force_close_ws_socket(self, ws_instance):
        """暴力切断底层的 TCP socket 并彻底关闭 ws_instance，防止泄露或阻塞"""
        if not ws_instance:
            return
        
        ws_instance.keep_running = False
        sock_obj = getattr(ws_instance, 'sock', None)
        
        if sock_obj:
            import socket
            # 依次清理底层的 raw socket 以及 websocket 封装的 socket 对象
            for s in (getattr(sock_obj, 'sock', None), sock_obj):
                if not s:
                    continue
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    s.close()
                except Exception:
                    pass
            ws_instance.sock = None
            
        try:
            ws_instance.close()
        except Exception:
            pass

    # -------------------- 底层透传接口 --------------------
    @property
    def _low_state(self):
        """底层硬件数据响应"""
        src = self.low_state_data if self.low_state_data else {}
        self._low_data = {
            "RobotStatus": src.get("RobotStatus", {
                "error_code": 0, "error_msg": "",
                "arm_connected": False, "arm_enable": False,
                "arm_status": 0, "arm_mode": -1,
            }),
            "RobotState": src.get("RobotState", {
                "joint_cmd_pos": [], "joint_cmd_vel": [], "joint_cmd_tau": [],
                "joint_pos": [], "joint_vel": [], "joint_tau": [],
            }),
            "gripperStatus": src.get("gripperStatus", {
                "gripper_error_code": 0, "gripper_error_msg": "",
                "gripper_connected": False, "gripper_enable": False,
                "gripper_status": 0, "gripper_mode": -1,
            }),
            "gripperState": src.get("gripperState", {
                "gripper_cmd_pos": 0.0, "gripper_cmd_vel": 0.0, "gripper_cmd_tau": 0.0,
                "gripper_pos": 0.0, "gripper_vel": 0.0, "gripper_tau": 0.0,
            }),
        }
        return self._low_data

    @property
    def low_status(self):
        """底层 RobotStatus 子字典"""
        return self._low_state.get("RobotStatus", {})

    @property
    def low_state(self):
        """底层 RobotState 子字典"""
        return self._low_state.get("RobotState", {})

    @property
    def low_gripper_status(self):
        """底层 gripperStatus 子字典"""
        return self._low_state.get("gripperStatus", {})

    @property
    def low_gripper_state(self):
        """底层 gripperState 子字典"""
        return self._low_state.get("gripperState", {})

    @property
    def low_error_code(self):
        """底层错误码"""
        return self.low_status.get("error_code", 0)

    @property
    def low_error_msg(self):
        """底层错误信息"""
        return self.low_status.get("error_msg", "")

    @property
    def low_arm_connected(self):
        """底层臂连接状态"""
        return self.low_status.get("arm_connected", False)

    @property
    def low_arm_enable(self):
        """底层臂使能状态"""
        return self.low_status.get("arm_enable", False)

    @property
    def low_arm_status(self):
        """底层臂状态"""
        return self.low_status.get("arm_status", 0)

    @property
    def low_arm_mode(self):
        """底层臂模式"""
        return self.low_status.get("arm_mode", -1)

    @property
    def low_joint_cmd_pos(self):
        """底层关节指令位置 (rad)"""
        return self.low_state.get("joint_cmd_pos", [])

    @property
    def low_joint_cmd_vel(self):
        """底层关节指令速度 (rad/s)"""
        return self.low_state.get("joint_cmd_vel", [])

    @property
    def low_joint_cmd_tau(self):
        """底层关节指令力矩 (N·m)"""
        return self.low_state.get("joint_cmd_tau", [])

    @property
    def low_joint_pos(self):
        """底层关节实际位置 (rad)"""
        return self.low_state.get("joint_pos", [])

    @property
    def low_joint_vel(self):
        """底层关节实际速度 (rad/s)"""
        return self.low_state.get("joint_vel", [])

    @property
    def low_joint_tau(self):
        """底层关节实际力矩 (N·m)"""
        return self.low_state.get("joint_tau", [])

    @property
    def low_gripper_cmd_pos(self):
        """底层夹爪指令位置"""
        return self.low_gripper_state.get("gripper_cmd_pos", 0.0)

    @property
    def low_gripper_cmd_vel(self):
        """底层夹爪指令速度"""
        return self.low_gripper_state.get("gripper_cmd_vel", 0.0)

    @property
    def low_gripper_cmd_tau(self):
        """底层夹爪指令力矩"""
        return self.low_gripper_state.get("gripper_cmd_tau", 0.0)

    @property
    def low_gripper_pos(self):
        """底层夹爪实际位置"""
        return self.low_gripper_state.get("gripper_pos", 0.0)

    @property
    def low_gripper_vel(self):
        """底层夹爪实际速度"""
        return self.low_gripper_state.get("gripper_vel", 0.0)

    @property
    def low_gripper_tau(self):
        """底层夹爪实际力矩"""
        return self.low_gripper_state.get("gripper_tau", 0.0)

    @property
    def low_gripper_connected(self):
        """底层夹爪连接状态"""
        return self.low_gripper_status.get("gripper_connected", False)

    @property
    def low_gripper_enable(self):
        """底层夹爪使能状态"""
        return self.low_gripper_status.get("gripper_enable", False)

    @property
    def low_gripper_err_code(self):
        """底层夹爪错误码"""
        return self.low_gripper_status.get("gripper_error_code", 0)

    @property
    def low_gripper_mode(self):
        """底层夹爪模式"""
        return self.low_gripper_status.get("gripper_mode", -1)

    def set_low_mode(self, flag=True) -> bool:
        """
        设置底层透传模式

        :param flag: bool, True 开启底层透传模式，False 关闭

        :return: bool, 执行是否成功
        """
        res = self.request({"command": "setLowMode", "arm_index": self.arm_index, "flag": flag})
        return res.get("recv") == "Task_Recieve"

    def low_pv_command(self, pos, vel) -> tuple:
        """
        发送底层位置速度(PV)控制指令并获取硬件状态

        :param pos: list, 目标关节位置 (rad)

        :param vel: list, 目标关节速度 (rad/s)

        :return: tuple, (success: bool, low_state: dict)
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel):
            return False, self._low_state
        pos, vel = list(pos), list(vel)
        if not self.__clip_joints(pos):
            return False, self._low_state
        if not self.__clip_joints_vel(vel):
            return False, self._low_state
        res = self.request({"command": "PVCommand", "arm_index": self.arm_index,
                           "joint_pos": pos, "joint_vel": vel})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_mit_command(self, pos, vel, tau, kp, kd) -> tuple:
        """
        发送底层MIT综合控制指令并获取硬件状态

        :param pos: list, 目标关节位置 (rad)

        :param vel: list, 目标关节速度 (rad/s)

        :param tau: list, 目标前馈力矩 (N·m)

        :param kp: list, 关节刚度参数

        :param kd: list, 关节阻尼参数

        :return: tuple, (success: bool, low_state: dict)
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel):
            return False, self._low_state
        if not self.__check_input_valid(tau) or not self.__check_input_valid(kp) or not self.__check_input_valid(kd):
            return False, self._low_state
        pos, vel = list(pos), list(vel)
        if not self.__clip_joints(pos):
            return False, self._low_state
        if not self.__clip_joints_vel(vel):
            return False, self._low_state
        dof = self.arm_dof
        for name, arr in [("tau", tau), ("kp", kp), ("kd", kd)]:
            if dof and len(arr) != dof:
                print(f"Error: {name} 维度({len(arr)})与臂自由度({dof})不一致")
                return False, self._low_state
        res = self.request({"command": "MITCommand", "arm_index": self.arm_index,
                           "joint_pos": pos, "joint_vel": vel,
                           "joint_tau": list(tau), "Kp": list(kp), "Kd": list(kd)})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_pf_command(self, pos, vel, tau) -> tuple:
        """
        发送底层位置力矩(PF)混合控制指令并获取硬件状态

        :param pos: list, 目标关节位置 (rad)

        :param vel: list, 目标关节速度 (rad/s)

        :param tau: list, 目标前馈力矩 (N·m)

        :return: tuple, (success: bool, low_state: dict)
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel) or not self.__check_input_valid(tau):
            return False, self._low_state
        pos, vel = list(pos), list(vel)
        if not self.__clip_joints(pos):
            return False, self._low_state
        if not self.__clip_joints_vel(vel):
            return False, self._low_state
        dof = self.arm_dof
        for name, arr in [("tau", tau)]:
            if dof and len(arr) != dof:
                print(f"Error: {name} 维度({len(arr)})与臂自由度({dof})不一致")
                return False, self._low_state
        res = self.request({"command": "PFCommand", "arm_index": self.arm_index,
                           "joint_pos": pos, "joint_vel": vel, "joint_tau": list(tau)})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_current_command(self, tau) -> tuple:
        """
        发送底层力矩(Current)指令并获取硬件状态

        :param tau: list, 目标关节力矩 (N·m)

        :return: tuple, (success: bool, low_state: dict)
        """
        if not self.__check_input_valid(tau):
            return False, self._low_state
        dof = self.arm_dof
        if dof and len(tau) != dof:
            print(f"Error: tau 维度({len(tau)})与臂自由度({dof})不一致")
            return False, self._low_state
        res = self.request({"command": "CURRENTCommand", "arm_index": self.arm_index,
                           "joint_tau": list(tau)})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_refresh(self) -> tuple:
        """
        主动刷新并获取当前最新的一帧底层硬件数据（无需下发控制指令）

        :return: tuple, (success: bool, low_state: dict)
        """
        res = self.request({"command": "Refresh", "arm_index": self.arm_index})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_set_end_effector_ctr(self, pos, vel, tau) -> tuple:
        """
        底层末端执行器（如夹爪等）控制指令

        :param pos: list, 目标位置

        :param vel: list, 目标速度

        :param tau: list, 目标力矩/力

        :return: tuple, (success: bool, low_state: dict)
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel) or not self.__check_input_valid(tau):
            return False, self._low_state
        pos, vel, tau = list(pos), list(vel), list(tau)
        dof = max(len(pos), len(vel), len(tau))
        if not self.__clip_eeff(dof, pos, vel, tau, control_motor=True):
            return False, self._low_state
        res = self.request({"command": "setEndEffectorCtr", "arm_index": self.arm_index,
                           "eeff_pos": pos, "eeff_vel": vel, "eeff_tau": tau})
        self.low_state_data = res
        return res.get("recv") == "Task_Recieve", self._low_state

    def low_set_robot_mode(self, mode) -> bool:
        """
        设置机器人底层的运行模式

        :param mode: int, 模式枚举值

        :return: bool, 执行是否成功
        """
        res = self.request({"command": "setRobotMode", "arm_index": self.arm_index, "mode": mode})
        return res.get("recv") == "Task_Recieve"

    def low_set_end_effector_mode(self, mode) -> bool:
        """
        设置末端执行器底层运行模式

        :param mode: int, 模式枚举值

        :return: bool, 执行是否成功
        """
        res = self.request({"command": "setEndEffectorMode", "arm_index": self.arm_index, "mode": mode})
        return res.get("recv") == "Task_Recieve"

    def low_set_servo_enable(self, status) -> bool:
        """
        控制底层伺服上/下使能

        :param status: bool, True 为上使能，False 为下使能

        :return: bool, 执行是否成功
        """
        res = self.request({"command": "setEnable", "arm_index": self.arm_index, "status": status})
        return res.get("recv") == "Task_Recieve"

    def low_reset(self, cnt=5) -> bool:
        """
        进行底层错误复位操作

        :param cnt: int, 尝试复位的最大次数，默认5次

        :return: bool, 执行是否成功
        """
        res = self.request({"command": "reset", "arm_index": self.arm_index, "cnt": cnt})
        return res.get("recv") == "Task_Recieve"

    def low_get_servo_status(self) -> dict:
        """
        主动获取底层伺服级状态，包括连接状态、温度、模式等详情信息

        :return: dict, 伺服状态字典，包含 mitKp/mitKd/pvVel/pfVel/isServoEnable/fsmMode/
                 isConnected/mosTemperature/motorTemperature/motorVBus/motorErrorCode/motorErrorMsg 等字段，
                 失败返回空字典
        """
        res = self.request({"command": "getServoStatus", "arm_index": self.arm_index})
        if res.get("recv") == "Task_Recieve":
            return res.get("ServoStatus", {})
        return {}

    def low_get_inverse_kine(self, pose, refer_pos, tool=-1) -> tuple:
        """
        执行底层逆运动学闭式求解

        :param pose: list, 目标笛卡尔空间位姿 [x, y, z, qx, qy, qz, qw]

        :param refer_pos: list, 参考关节角度（多解情况下选优基准）

        :param tool: int, 工具号或坐标系编号，默认 -1 表示当前使用的工具号

        :return: tuple, (success: bool, tool: int, joint_pos: list)，求解出的目标关节角度
        """
        req = {"command": "getInverseKine", "arm_index": self.arm_index,
               "pose": list(pose), "refer_pos": list(refer_pos)}
        if tool >= 0:
            req["tool"] = tool
        res = self.request(req)
        if res.get("recv") == "Task_Recieve":
            return True, res.get("tool", tool), res.get("joint_pos", [])
        return False, 0, []

    def low_get_forward_kine(self, joint_pos, tool=-1) -> tuple:
        """
        执行底层正运动学计算

        :param joint_pos: list, 关节角度组合

        :param tool: int, 工具号或坐标系编号，默认 -1 表示当前使用的工具号

        :return: tuple, (success: bool, tool: int, pose: list)，求解出的笛卡尔位姿 [x, y, z, qx, qy, qz, qw]
        """
        req = {"command": "getForwardKine", "arm_index": self.arm_index, "joint_pos": list(joint_pos)}
        if tool >= 0:
            req["tool"] = tool
        res = self.request(req)
        if res.get("recv") == "Task_Recieve":
            return True, res.get("tool", tool), res.get("pose", [])
        return False, 0, []

    def low_get_dynamics(self, joint_pos, joint_vel, joint_acc) -> tuple:
        """
        利用底层动力学库计算机器人惯性矩阵(M)、科里奥利力与离心力(C)、重力(G)分量

        :param joint_pos: list, 关节角度

        :param joint_vel: list, 关节角速度

        :param joint_acc: list, 关节角加速度

        :return: tuple, (success: bool, tool: int, m_force: list, c_force: list, g_force: list)
        """
        res = self.request({"command": "getDynamics", "arm_index": self.arm_index,
                           "joint_pos": list(joint_pos), "joint_vel": list(joint_vel),
                           "joint_acc": list(joint_acc)})
        if res.get("recv") == "Task_Recieve":
            return True, res.get("tool", 0), res.get("m_force", []), res.get("c_force", []), res.get("g_force", [])
        return False, 0, [], [], []

    def low_get_jacobian(self, joint_pos) -> tuple:
        """
        获取底层雅可比矩阵(Jacobian)

        :param joint_pos: list, 当前关节角度

        :return: tuple, (success: bool, tool: int, matrix: list[list[float]])，
                 matrix 为二维列表，形状 (rows, cols)，即 matrix[i][j] 表示第 i 行第 j 列元素；
                 失败返回 (False, 0, [])
        """
        res = self.request({"command": "getJacobian", "arm_index": self.arm_index,
                           "joint_pos": list(joint_pos)})
        if res.get("recv") == "Task_Recieve":
            rows = res.get("rows", 0)
            cols = res.get("cols", 0)
            flat = res.get("jacobian", [])
            matrix = [flat[i * cols:(i + 1) * cols] for i in range(rows)] if rows and cols else []
            return True, res.get("tool", 0), matrix
        return False, 0, []

    def low_get_nullspace(self, joint_pos, tolerance) -> tuple:
        """
        获取底层零空间(Nullspace)矩阵

        :param joint_pos: list, 当前关节角度

        :param tolerance: float, 求解公差值

        :return: tuple, (success: bool, tool: int, matrix: list[list[float]])，
                 matrix 为二维列表，形状 (rows, cols)；失败返回 (False, 0, [])
        """
        res = self.request({"command": "getNullspace", "arm_index": self.arm_index,
                           "joint_pos": list(joint_pos), "tolerance": tolerance})
        if res.get("recv") == "Task_Recieve":
            rows = res.get("rows", 0)
            cols = res.get("cols", 0)
            flat = res.get("nullspace", [])
            matrix = [flat[i * cols:(i + 1) * cols] for i in range(rows)] if rows and cols else []
            return True, res.get("tool", 0), matrix
        return False, 0, []


if __name__ == "__main__":
    carm = Carm()
    carm.track_joint(carm.joint_pos)
    print(1)
    carm.move_joint(carm.joint_pos)
    print(2)
    carm.track_pose(carm.cart_pose)
    print(3)
    carm.move_pose(carm.cart_pose)
    print(4)
