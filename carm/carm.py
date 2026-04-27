import websocket
import threading
import json
import uuid
import time
import math

class Carm:
    def __init__(self, addr="10.42.0.101", arm_index=0):
        self.state = None
        self.last_msg = None
        self.ws = None
        self.arm_index = arm_index
        self.addr = addr
        self.port = 8090
        self._retry_delay = 1.0
        self._max_delay = 10.0

        self.ops = {
            "webSendRobotState": lambda msg: self.__cbk_status(msg),
            "taskFinished": lambda msg: self.__cbk_taskfinish(msg),
            "onCarmError": lambda msg: self.__cbk_error(msg)
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
    def connect(self, addr=None, port=None, timeout=1):
        """
        连接到机械臂控制器。
        :param addr: IP 地址，默认使用初始化时的地址
        :param port: 端口，默认 8090
        :param timeout: 连接超时（秒），默认 1
        :return: True 表示连接成功，False 表示失败
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

    def disconnect(self):
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
            old_reader.join() # 无 timeout，绝对保证断联后后台干净
            
        self.open_ready.clear()
        self.ws = None # 完全切断残余连接对象引用
        print("Disconnected cleanly.")

    def is_connected(self):
        """检查当前是否已连接"""
        return self.open_ready.is_set()

    # -------------------- 属性（状态获取） --------------------
    @property
    def _arm_state(self):
        """内部方法：安全获取当前手臂的状态字典"""
        if not self.state or "arm" not in self.state or len(self.state["arm"]) <= self.arm_index:
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
    def version(self):
        """获取控制器软件版本"""
        return self.request({
            "command": "getArmIntrinsicProperties",
            "arm_index": self.arm_index,
            "type": "version"
        })

    def get_limits(self):
        """获取关节限位、最大速度、加速度等参数"""
        return self.request({
            "command": "getJointParams",
            "arm_index": self.arm_index
        })

    def get_eeff_config(self):
        """获取末端执行器配置"""
        res = self.request({
            "command": "getEeffParams",
            "arm_index": self.arm_index
        })
        if res.get("params") == None:
            if self.end_effector_type == "gripper":
                res["params"] = {
                    "eeff_dof": self.end_effector_dof,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [0.077]*self.end_effector_dof,
                    "eeff_vel": [0.0]*self.end_effector_dof,
                    "eeff_tau": [100.0]*self.end_effector_dof
                }
            elif self.end_effector_type == "hand":
                res["params"] = {
                    "eeff_dof": self.end_effector_dof,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [255.0]*self.end_effector_dof,
                    "eeff_vel": [255.0]*self.end_effector_dof,
                    "eeff_tau": [255.0]*self.end_effector_dof
                }
            else:
                res["params"] = {
                    "eeff_dof": self.end_effector_dof,
                    "eeff_lower": [0.0]*self.end_effector_dof,
                    "eeff_upper": [255.0]*self.end_effector_dof,
                    "eeff_vel": [255.0]*self.end_effector_dof,
                    "eeff_tau": [255.0]*self.end_effector_dof
                }
        return res

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
    def gripper_state(self):
        """夹爪状态（-1 未连接/无夹爪，0 未使能，1 正常）"""
        eeff = self._arm_state.get("eeff", {})
        if not eeff.get("is_connect", False) or eeff.get("eeff_type") != "gripper":
            return -1
        return eeff.get("eeff_state", -1)

    @property
    def gripper_pos(self):
        """实际夹爪位置（单位：m）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "gripper":
            return 0.0
        pos = eeff.get("eeff_pos", [])
        return pos[0] if pos else 0.0

    @property
    def gripper_tau(self):
        """实际夹爪力矩（单位：N）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "gripper":
            return 0.0
        tau = eeff.get("eeff_tau", [])
        return tau[0] if tau else 0.0

    @property
    def plan_gripper_pos(self):
        """规划夹爪位置（单位：m）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "gripper":
            return 0.0
        pos = eeff.get("eeff_plan_pos", [])
        return pos[0] if pos else 0.0

    @property
    def plan_gripper_tau(self):
        """规划夹爪力矩（单位：N）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "gripper":
            return 0.0
        tau = eeff.get("eeff_plan_tau", [])
        return tau[0] if tau else 0.0

    @property
    def hand_state(self):
        """灵巧手状态（-1 未连接/无灵巧手，0 未使能，1 正常）"""
        eeff = self._arm_state.get("eeff", {})
        if not eeff.get("is_connect", False) or eeff.get("eeff_type") != "hand":
            return -1
        return eeff.get("eeff_state", -1)

    @property
    def hand_pos(self):
        """实际灵巧手位置（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_pos", [])

    @property
    def hand_vel(self):
        """实际灵巧手速度（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_vel", [])

    @property
    def hand_tau(self):
        """实际灵巧手力矩（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_tau", [])

    @property
    def plan_hand_pos(self):
        """规划灵巧手位置（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_plan_pos", [])

    @property
    def plan_hand_vel(self):
        """规划灵巧手速度（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_plan_vel", [])

    @property
    def plan_hand_tau(self):
        """规划灵巧手力矩（列表）"""
        eeff = self._arm_state.get("eeff", {})
        if eeff.get("eeff_type") != "hand":
            return []
        return eeff.get("eeff_plan_tau", [])

    # -------------------- 控制命令 --------------------
    def set_ready(self, timeout_ms=3000):
        """
        将机械臂设置为就绪状态（错误清除、伺服上使能、切换到位置模式）。
        :param timeout_ms: 超时时间（毫秒），默认 3000ms
        :return: True 表示就绪成功，False 表示失败
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
            self.limit = self.get_limits()["params"]  # 更新配置
            self.eeff_limit = self.get_eeff_config()["params"]  # 更新末端配置
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
            self.limit = self.get_limits()["params"]
            self.eeff_limit = self.get_eeff_config()["params"]
            return True
        else:
            return False

    def set_servo_enable(self, enable=True):
        """
        设置伺服使能
        :param enable: True 使能，False 下使能
        """
        return self.request({
            "command": "setServoEnable",
            "arm_index": self.arm_index,
            "enable": enable
        })

    def set_control_mode(self, mode=1):
        """
        设置控制模式
        :param mode: 0-IDLE, 1-点位, 2-MIT, 3-拖动, 4-力位混合
        """
        mode = self.__clip(mode, 0, 4)
        return self.request({
            "command": "setControlMode",
            "arm_index": self.arm_index,
            "mode": mode
        })

    def set_passthrough_data(self, mode, can_id, data):
        """
        设置透传数据
        :param mode: 模式
        :param can_id: CAN ID
        :param data: 透传数据（字节列表或十六进制字符串等，需底层支持）
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
        if res.get("ret", 0) == 1 and (mode == 1 or mode == 2):
            ret_data = res.get("data", {})
            return ret_data.get("can_id", can_id), bytes.fromhex(ret_data.get("data", ""))
        return res
    
    def set_end_effector(self, dof, pos, vel, tau):
        """
        设置末端执行器（夹爪）的位置、速度、力矩，指定自由度 dof。
        输入可以是单个数值或列表，长度不足 dof 则补零，超出则截断。
        :param dof: 末端执行器自由度（整数）
        :param pos: 位置值或列表（单位：m or rad）
        :param vel: 速度值或列表（单位：m/s or rad/s）
        :param tau: 力矩值或列表（单位：N）
        :return: 请求响应
        """
        if not self.__check_input_valid(pos) or not self.__check_input_valid(vel) or not self.__check_input_valid(tau):
            print(f"Error: set_end_effector input contains NaN or Inf")
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}

        self.__clip_eeff(dof, pos, vel, tau)
        return self.request({
            "command": "setEffectorCtr",
            "arm_index": self.arm_index,
            "pos": pos,
            "vel": vel,
            "tau": tau
        })

    def set_gripper(self, pos, tau=10):
        """设置夹爪位置和力矩（pos单位：m，tau单位：N）"""
        return self.set_end_effector(1, [pos], [0.0], [tau])

    def set_hand(self, pos, tau, vel):
        """
        设置灵巧手位置、力矩和速度
        :param pos: 灵巧手位置或列表
        :param tau: 灵巧手力矩或列表
        :param vel: 灵巧手速度或列表
        :return: 请求响应
        """
        dof = max(len(pos) if isinstance(pos, list) else 0,
                  len(tau) if isinstance(tau, list) else 0,
                  len(vel) if isinstance(vel, list) else 0)
        if dof == 0 or len(tau)!= dof or len(vel)!= dof:
            print(f"Error: set_hand dof is zero, invalid input.")
            return {'recv': 'Task_Refuse', 'errMsg': 'Invalid input for hand control'}
        return self.set_end_effector(dof, pos, vel, tau)

    def set_tool_index(self, index):
        """
        设置当前工具号。
        :param index: 工具索引（整数，通常从 0 开始）
        :return: 请求响应，可检查 recv 字段是否为 "Task_Recieve"
        """
        if not self.__check_input_valid(index):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        return self.request({
            "command": "setToolData",
            "operation": "change",
            "index": index,
            "arm_index": self.arm_index
        })

    @property
    def tool_index(self):
        """
        获取当前工具号。
        :return: 当前工具索引（整数），若状态未更新或无此字段返回 0
        """
        return self._arm_state.get("tool", 0)
        
    def get_tool_coordinate(self, tool):
        """获取指定工具的坐标系（工具末端相对法兰的位姿）"""
        if not self.__check_input_valid(tool):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        return self.request({
            "command": "getCoordinate",
            "arm_index": self.arm_index,
            "type": "tool",
            "index": tool
        })

    def set_collision_config(self, flag=True, level=10):
        """
        设置碰撞检测
        :param flag: 开关
        :param level: 灵敏度等级 0~2
        """
        level = self.__clip(level, 0, 2)
        return self.request({
            "command": "setCollisionConfig",
            "arm_index": self.arm_index,
            "flag": flag,
            "level": level
        })

    def stop(self, type=0):
        """
        停止机械臂
        :param type: 0-暂停, 1-停止, 2-禁用, 3-紧急停止
        """
        type = self.__clip(type, 0, 3)
        stop_id = ["SIG_ARM_PAUSE", "SIG_ARM_STOP", "SIG_ARM_DISABLE", "SIG_EMERGENCY_STOP"]
        return self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": stop_id[type],
            "step_cnt": 5
        })

    def stop_task(self, at_once=False):
        """
        停止当前任务
        :param at_once: True 立即停止，False 完成当前任务后停止
        """
        return self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": "SIG_TASK_STOP",
            "stop_at_once": at_once
        })

    def recover(self):
        """恢复机械臂（退出暂停/急停）"""
        return self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": "SIG_ARM_RECOVER",
            "step_cnt": 5
        })

    def clean_carm_error(self):
        """清除控制器错误"""
        return self.request({
            "command": "setControllerErrorReset",
            "arm_index": self.arm_index
        })

    def set_speed_level(self, level=5.0, response_level=20):
        """
        设置速度等级
        :param level: 速度等级 0~10
        :param response_level: 过渡周期数 1~10000
        """
        if not self.__check_input_valid(level) or not self.__check_input_valid(response_level):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        level = self.__clip(level, 0, 10)
        response_level = self.__clip(response_level, 1, 10000)
        return self.request({
            "command": "setSpeedLevel",
            "arm_index": self.arm_index,
            "level": level,
            "response_level": response_level
        })

    # -------------------- 运动接口 --------------------
    def track_joint(self, pos, end_effector=None):
        """
        关节空间轨迹跟踪（周期性发送目标关节位置）
        :param pos: 目标关节位置
        :param end_effector: 夹爪位置（单位：m）
        :param tau: 夹爪力矩（单位：N）
        """
        if not self.__check_input_valid(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__clip_joints(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Joint size error'}
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

    def track_pose(self, pos, end_effector=None):
        """
        笛卡尔空间轨迹跟踪（周期性发送目标位姿）
        :param pos: 目标笛卡尔位姿 [x, y, z, qx, qy, qz, qw]
        :param end_effector: 夹爪位置（单位：m）
        :param tau: 夹爪力矩（单位：N）
        """
        _pos = list(pos)
        if not self.__check_input_valid(_pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        
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

    def move_joint(self, pos, tm=-1, is_sync=True, tool=0):
        """
        关节点的关节空间点到点运动
        :param pos: 目标关节位置
        :param tm: 运动时间（未使用，预留）
        :param is_sync: 是否同步等待
        :param tool: 工具号
        """
        if not self.__check_input_valid(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__clip_joints(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Joint size error'}
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
        return res

    def move_pose(self, pos, tm=-1, is_sync=True, tool=0):
        """
        笛卡尔点的关节空间点到点运动
        :param pos: 目标位姿 [x, y, z, qx, qy, qz, qw]
        :param tm: 运动时间（未使用，预留）
        :param is_sync: 是否同步等待
        :param tool: 工具号
        """
        if not self.__check_input_valid(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__check_normalized(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Quaternion not normalized'}
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
        return res

    def move_line_pose(self, pos, is_sync=True, tool=0):
        """
        笛卡尔点的空间直线运动
        :param pos: 目标位姿 [x, y, z, qx, qy, qz, qw]
        :param is_sync: 是否同步等待
        :param tool: 工具号
        """
        if not self.__check_input_valid(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__check_normalized(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Quaternion not normalized'}
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
        return res

    def move_line_joint(self, pos, is_sync=True, tool=0):
        """
        关节点的空间直线运动
        :param pos: 目标关节位置
        :param is_sync: 是否同步等待
        :param tool: 工具号
        """
        if not self.__check_input_valid(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__clip_joints(pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Joint size error'}
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
        return res

    def move_flow_pose(self, target_pos, line_theta_weight=0.5, accuracy=0.0001, is_sync=True, tool=0):
        """
        笛卡尔雅可比迭代运动（TASK_FLOW）
        :param target_pos: 目标位姿 [x, y, z, qx, qy, qz, qw]
        :param line_theta_weight: 位置与姿态的权重（0~1）
        :param accuracy: 收敛精度
        :param is_sync: 是否同步等待
        :param tool: 工具号
        """
        if not self.__check_input_valid(target_pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
        if not self.__check_normalized(target_pos):
            return {'recv': 'Task_Refuse', 'errMsg': 'Quaternion not normalized'}
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
                "line_theta_weight": line_theta_weight
            }
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res

    def move_toppra(self, targets, speed=100, tool=0, is_joint_val=True, is_sync=True):
        """
        基于 TOPPRA 的多点轨迹运动
        :param targets: 目标轨迹点列表，关节位置列表或位姿列表 [x, y, z, qx, qy, qz, qw]
        :param speed: 速度百分比
        :param tool: 工具号
        :param is_joint_val: True 表示关节空间目标，False 表示笛卡尔空间目标
        :param is_sync: 是否同步等待
        """
        if not targets:
            return {'recv': 'Task_Refuse', 'errMsg': 'Targets cannot be empty'}
        
        if not isinstance(targets[0], list):
            targets = [targets]
        
        for p in targets:
            if not self.__check_input_valid(p):
                return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}
            if is_joint_val:
                if not self.__clip_joints(p):
                    return {'recv': 'Task_Refuse', 'errMsg': 'Joint size error'}
            else:
                if not self.__check_normalized(p):
                    return {'recv': 'Task_Refuse', 'errMsg': 'Quaternion not normalized'}
        
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
        return res

    def move_joint_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True):
        """
        关节轨迹运动（使用基于 TOPPRA 的轨迹规划）
        :param target_traj: 目标关节位置轨迹列表
        :param gripper_pos: 夹爪位置（暂不生效，预留）
        :param stamps: 时间戳（不生效，TOPPRA 仅需要路径点）
        :param is_sync: 是否同步等待
        """
        return self.move_toppra(target_traj, is_joint_val=True, is_sync=is_sync)

    def move_pose_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True):
        """
        位姿轨迹运动（使用基于 TOPPRA 的轨迹规划）
        :param target_traj: 目标笛卡尔位姿轨迹列表
        :param gripper_pos: 夹爪位置（暂不生效，预留）
        :param stamps: 时间戳（不生效，TOPPRA 仅需要路径点）
        :param is_sync: 是否同步等待
        """
        return self.move_toppra(target_traj, is_joint_val=False, is_sync=is_sync)

    # -------------------- 示教接口 --------------------
    def trajectory_teach(self, off_on, name=""):
        """
        开始/停止示教录制
        :param off_on: True 开始录制，False 停止录制
        :param name: 轨迹名称（如 "20250918175001.my_traj"）
        """
        return self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 1 if off_on else 0,
            "name": name
        })

    def trajectory_recorder(self, name, is_sync=True):
        """
        复现指定名称的轨迹
        :param name: 轨迹名称
        :param is_sync: 是否同步等待
        """
        res = self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 2,
            "name": name
        })
        if is_sync and res.get("recv") == "Task_Recieve":
            self.__wait_task(res.get("task_key"))
        return res

    def check_teach(self):
        """
        获取已录制的轨迹列表，只有正则匹配上的 '20250918175001.self_name.json' 才会被返回
        :return: 轨迹列表的字符串数组
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

    # -------------------- 运动学 --------------------
    def inverse_kine(self, cart_pose, ref_joints, tool=0):
        """
        逆运动学求解
        :param cart_pose: 目标位姿 [x, y, z, qx, qy, qz, qw]（单个或列表）
        :param ref_joints: 参考关节角（单个或列表）
        :param tool: 工具号
        :return: 返回包含关节角的响应
        """
        if not self.__check_input_valid(cart_pose) or not self.__check_input_valid(ref_joints):
            return {'recv': 'Task_Refuse', 'errMsg': 'Input contains NaN or Inf'}

        if not isinstance(cart_pose[0], list):
            cart_pose = [cart_pose]
            ref_joints = [ref_joints]
        assert len(cart_pose) == len(ref_joints)

        for p in cart_pose:
            if not self.__check_normalized(p):
                return {'recv': 'Task_Refuse', 'errMsg': 'Quaternion not normalized'}
        
        data = {"tool": tool, "point_cnt": len(cart_pose)}
        for i in range(len(ref_joints)):
            data[f"point{i+1}"] = cart_pose[i]
            data[f"refer{i+1}"] = ref_joints[i]
        return self.request({
            "command": "getKinematics",
            "task_id": "inverse",
            "arm_index": self.arm_index,
            "data": data
        })

    def forward_kine(self, joint_pos, tool=0):
        """
        正运动学求解
        :param joint_pos: 关节角（单个或列表）
        :param tool: 工具号
        :return: 位姿列表或单个位姿 [x, y, z, qx, qy, qz, qw]，失败返回 None
        """
        if not self.__check_input_valid(joint_pos):
            print(f"Error: forward_kine input contains NaN or Inf")
            return None

        is_list = True
        if not isinstance(joint_pos[0], list):
            joint_pos = [joint_pos]
            is_list = False

        for v in joint_pos:
            if not self.__clip_joints(v):
                print(f"Error: forward_kine joint size error for {v}")
                return None

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
            if ret["recv"] == "Task_Recieve":
                if is_list:
                    points = []
                    for i in range(len(joint_pos)):
                        points.append(ret["data"][f"point{i+1}"])
                    return points
                else:
                    return ret["data"]["point1"]
        except Exception as e:
            print(f"Error parsing forward_kine response: {e}")
            return None

    # -------------------- 回调注册 --------------------
    def on_error(self, callback):
        """
        注册错误回调函数
        :param callback: 函数签名 fn(error_code, error_message)
        """
        self.ops["onCarmError"] = callback

    def on_task_finish(self, callback):
        """
        注册任务完成回调函数
        :param callback: 函数签名 fn(task_key)
        """
        self.ops["taskFinished"] = callback

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
        if "arm" not in message:
            return

        self.state = message

        # 全局错误解析
        if message.get("error", 0) != 0 or message.get("errMsg", ""):
            error_info = {
                "command": "onCarmError",
                "error": message.get("error"),
                "errMsg": message.get("errMsg"),
                "error_arm_index": message.get("error_arm_index", -1)
            }
            self.ops["onCarmError"](error_info)
            self.__abort_all_tasks()  # 中断所有等待

        # 任务完成解析（只处理当前臂）
        if len(message["arm"]) > self.arm_index:
            arm_json = message["arm"][self.arm_index]
            if "task" in arm_json:
                task_info = arm_json["task"]
                if "last_task_key" in task_info and task_info["last_task_key"]:
                    task_key = task_info["last_task_key"]
                    self.ops["taskFinished"](task_key)
                    # 触发特定的 task_pool 事件
                    if task_key in self.task_pool:
                        self.task_pool[task_key].set()
                            

    def __cbk_taskfinish(self, message):
        print(f"Task finished callback received: {message}")

    def __cbk_error(self, message):
        print(f"Error callback received: {message}")

    def __on_open(self, ws):
        self.open_ready.set()
        self._retry_delay = 1.0  # 连接成功后重置重连延迟
        print("Connected successfully.")
        
        def fetch_limits():
            try:
                if self.is_connected():
                    res = self.get_limits()
                    if res and "params" in res:
                        self.limit = res["params"]
                    eeff_res = self.get_eeff_config()
                    if eeff_res and "params" in eeff_res:
                        self.eeff_limit = eeff_res["params"]
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

    def __clip_joints(self, joints):
        if not self.limit:
            return True
            
        lower = self.limit.get('limit_lower', [])
        upper = self.limit.get("limit_upper", [])
        
        if len(lower) != len(joints) or len(upper) != len(joints):
            return False

        for i, v in enumerate(joints):
            joints[i] = self.__clip(v, lower[i], upper[i])
        return True
    
    def __clip_eeff(self, dof, eeff_pos, eeff_vel, eeff_tau):
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

        if not self.eeff_limit:
            return True
        
        if dof != self.eeff_limit.get("eeff_dof", 0):
            return False

        lower = self.eeff_limit.get("eeff_lower", [])
        upper = self.eeff_limit.get("eeff_upper", [])
        vel = self.eeff_limit.get("eeff_vel", [])
        tau = self.eeff_limit.get("eeff_tau", [])

        # 自动找齐
        auto_pad(eeff_pos, dof)
        auto_pad(eeff_tau, dof)
        auto_pad(eeff_vel, dof)
        # 自动限制
        for i, v in enumerate(eeff_pos):
            eeff_pos[i] = self.__clip(v, lower[i], upper[i])
        for i, v in enumerate(eeff_tau):
            eeff_tau[i] = self.__clip(v, 0.0, tau[i])
        for i, v in enumerate(eeff_vel):
            eeff_vel[i] = self.__clip(v, 0.0, vel[i])
        return True


    def __check_input_valid(self, values):
        """检查输入值是否包含 NaN 或 Inf"""
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