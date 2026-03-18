import websocket
import threading
import json
import uuid
import time

class Carm:
    def __init__(self, addr="10.42.0.101", arm_index=0):
        self.state = None
        self.last_msg = None
        self.arm_index = arm_index
        self.addr = addr
        self.port = 8090

        self.ops = {
            "webSendRobotState": lambda msg: self.__cbk_status(msg),
            "taskFinished": lambda msg: self.__cbk_taskfinish(msg),
            "onCarmError": lambda msg: print("Error:", msg)
        }
        self.res_pool = {}
        self.task_event = threading.Event()
        self.open_ready = threading.Event()

        # 自动连接
        self._create_connection()
        self.limit = self.get_limits()["params"]

    def _create_connection(self):
        """创建 WebSocket 连接（内部使用）"""
        while not self.open_ready.wait(1):
            self.ws = websocket.WebSocketApp(
                f"ws://{self.addr}:{self.port}",
                on_open=lambda ws: self.__on_open(ws),
                on_close=lambda ws, code, close_msg: self.__on_close(ws, code, close_msg),
                on_message=lambda ws, msg: self.__on_message(ws, msg),
            )
            self.reader = threading.Thread(target=self.__recv_loop, daemon=True).start()

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
        self.open_ready.clear()
        self._create_connection()
        return self.open_ready.wait(timeout)

    def disconnect(self):
        """断开与机械臂控制器的连接"""
        if self.ws:
            self.ws.close()
        self.open_ready.clear()

    def is_connected(self):
        """检查当前是否已连接"""
        return self.open_ready.is_set()

    # -------------------- 属性（状态获取） --------------------
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

    @property
    def joint_pos(self):
        """实际关节位置（弧度）"""
        return self.state["arm"][self.arm_index]["reality"]["pose"]

    @property
    def joint_vel(self):
        """实际关节速度"""
        return self.state["arm"][self.arm_index]["reality"]["vel"]

    @property
    def joint_tau(self):
        """实际关节力矩"""
        return self.state["arm"][self.arm_index]["reality"]["torque"]

    @property
    def plan_joint_pos(self):
        """规划关节位置"""
        return self.state["arm"][self.arm_index]["plan"]["pose"]

    @property
    def plan_joint_vel(self):
        """规划关节速度"""
        return self.state["arm"][self.arm_index]["plan"]["vel"]

    @property
    def plan_joint_tau(self):
        """规划关节力矩"""
        return self.state["arm"][self.arm_index]["plan"]["torque"]

    @property
    def cart_pose(self):
        """当前实际笛卡尔位姿（x,y,z,qw,qx,qy,qz）"""
        return self.state["arm"][self.arm_index]["pose"]

    @property
    def plan_cart_pose(self):
        """当前规划笛卡尔位姿（直接取自上报数据）"""
        return self.state["arm"][self.arm_index]["plan"]["cart_pose"]

    @property
    def joint_external_tau(self):
        """关节外力矩"""
        return self.state["arm"][self.arm_index].get("joint_external_tau", [])

    @property
    def cart_external_force(self):
        """笛卡尔外力（fx,fy,fz,tx,ty,tz）"""
        return self.state["arm"][self.arm_index].get("cart_external_force", [])

    # -------------------- 末端执行器（夹爪）属性 --------------------
    @property
    def gripper_state(self):
        """夹爪状态（-1 未连接/无夹爪，0 未使能，1 正常）"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        if not eeff.get("is_connect", False):
            return -1
        return eeff.get("eeff_state", -1)

    @property
    def gripper_pos(self):
        """实际夹爪位置（单位：米）"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        pos = eeff.get("eeff_pos", [])
        return pos[0] if pos else 0.0

    @property
    def gripper_vel(self):
        """实际夹爪速度"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        vel = eeff.get("eeff_vel", [])
        return vel[0] if vel else 0.0

    @property
    def gripper_tau(self):
        """实际夹爪力矩"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        tau = eeff.get("eeff_tau", [])
        return tau[0] if tau else 0.0

    @property
    def plan_gripper_pos(self):
        """规划夹爪位置"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        pos = eeff.get("eeff_plan_pos", [])
        return pos[0] if pos else 0.0

    @property
    def plan_gripper_vel(self):
        """规划夹爪速度"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        vel = eeff.get("eeff_plan_vel", [])
        return vel[0] if vel else 0.0

    @property
    def plan_gripper_tau(self):
        """规划夹爪力矩"""
        eeff = self.state["arm"][self.arm_index].get("eeff", {})
        tau = eeff.get("eeff_plan_tau", [])
        return tau[0] if tau else 0.0

    # -------------------- 控制命令 --------------------
    def set_ready(self):
        """将机械臂设置为就绪状态（错误清除、伺服上使能、切换到位置模式）"""
        while self.state is None:
            time.sleep(0.1)
        arm = self.state["arm"][self.arm_index]
        if arm["fsm_state"] == "POSITION" or arm["fsm_state"] == "MIT":
            return True
        if arm["fsm_state"] == "ERROR":
            self.clean_carm_error()
        if arm["fsm_state"] == "IDLE":
            self.set_servo_enable(True)
        return self.set_control_mode(1)

    def set_servo_enable(self, enable=True):
        """设置伺服使能"""
        return self.request({
            "command": "setServoEnable",
            "arm_index": self.arm_index,
            "enable": enable
        })

    def set_control_mode(self, mode=1):
        """设置控制模式：0-IDLE, 1-点位, 2-MIT, 3-拖动, 4-力位混合"""
        return self.request({
            "command": "setControlMode",
            "arm_index": self.arm_index,
            "mode": mode
        })

    def set_end_effector(self, pos, tau):
        """设置末端执行器（夹爪）的位置和力矩（兼容旧接口）"""
        return self.request({
            "command": "setEffectorCtr",
            "arm_index": self.arm_index,
            "pos": pos,
            "tau": tau
        })

    def set_gripper(self, pos, tau=10):
        """设置夹爪位置和力矩（单位：米，牛）"""
        pos = self.__clip(pos, 0, 0.08)
        tau = self.__clip(tau, 0, 100)
        return self.set_end_effector(pos, tau)

    def get_tool_coordinate(self, tool):
        """获取指定工具的坐标系（工具末端相对法兰的位姿）"""
        return self.request({
            "command": "getCoordinate",
            "arm_index": self.arm_index,
            "type": "tool",
            "index": tool
        })

    def set_collision_config(self, flag=True, level=10):
        """设置碰撞检测（flag 开关，level 灵敏度等级 0~2）"""
        return self.request({
            "command": "setCollisionConfig",
            "arm_index": self.arm_index,
            "flag": flag,
            "level": level
        })

    def stop(self, type=0):
        """
        停止机械臂
        type: 0-暂停, 1-停止, 2-禁用, 3-紧急停止
        """
        stop_id = ["SIG_ARM_PAUSE", "SIG_ARM_STOP", "SIG_ARM_DISABLE", "SIG_EMERGENCY_STOP"]
        return self.request({
            "command": "stopSignals",
            "arm_index": self.arm_index,
            "stop_id": stop_id[type],
            "step_cnt": 5
        })

    def stop_task(self, at_once=False):
        """停止当前任务，at_once=True 立即停止，False 完成当前任务后停止"""
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
        """设置速度等级（0~10），response_level 为过渡周期数（1~10000）"""
        return self.request({
            "command": "setSpeedLevel",
            "arm_index": self.arm_index,
            "level": level,
            "response_level": response_level
        })

    def set_debug(self, flag):
        """设置调试模式（模拟运行）"""
        return self.request({
            "command": "setDebugMode",
            "arm_index": self.arm_index,
            "trigger": flag
        })

    # -------------------- 运动接口 --------------------
    def track_joint(self, pos, end_effector=None, tau=20):
        """关节空间轨迹跟踪（周期性发送目标关节位置）"""
        pos = self.__clip_joints(pos)
        req = {
            "command": "trajectoryTrackingTasks",
            "task_id": "TASK_TRACKING",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"way_point": pos}
        }
        if end_effector is not None:
            self.set_end_effector(end_effector, tau)
        return self.request(req)

    def track_pose(self, pos, end_effector=None, tau=20):
        """笛卡尔空间轨迹跟踪（周期性发送目标位姿）"""
        req = {
            "command": "trajectoryTrackingTasks",
            "task_id": "TASK_TRACKING",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"way_point": pos}
        }
        if end_effector is not None:
            self.set_end_effector(end_effector, tau)
        return self.request(req)

    def move_joint(self, pos, tm=-1, sync=True, tool=0):
        """关节空间点到点运动（TASK_MOVJ）"""
        pos = self.__clip_joints(pos)
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"tool": tool, "target_pos": pos, "speed": 100}
        })
        if sync and res["recv"] == "Task_Recieve":
            self.__wait_task(res["task_key"])
        return res

    def move_pose(self, pos, tm=-1, sync=True, tool=0):
        """笛卡尔空间点到点运动（TASK_MOVJ 空间标志为 1）"""
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"tool": tool, "target_pos": pos, "speed": 100}
        })
        if sync and res["recv"] == "Task_Recieve":
            self.__wait_task(res["task_key"])
        return res

    def move_line_pose(self, pos, speed=100, sync=True, tool=0):
        """笛卡尔空间直线运动（TASK_MOVL 空间标志为 1）"""
        # 注意：这里传入的是笛卡尔位姿，point_type=1，直接发送位姿
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVL",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 1},
            "data": {"tool": tool, "point": pos, "speed": speed}
        })
        if sync and res["recv"] == "Task_Recieve":
            self.__wait_task(res["task_key"])
        return res

    def move_line_joint(self, pos, speed=100, sync=True, tool=0):
        """关节空间直线运动（TASK_MOVL 空间标志为 0）"""
        pos = self.__clip_joints(pos)
        res = self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVL",
            "task_level": "Task_General",
            "arm_index": self.arm_index,
            "point_type": {"space": 0},
            "data": {"tool": tool, "point": pos, "speed": speed}
        })
        if sync and res["recv"] == "Task_Recieve":
            self.__wait_task(res["task_key"])
        return res

    def move_flow_pose(self, target_pos, line_theta_weight=0.5, accuracy=0.0001, sync=True, tool=0):
        """
        笛卡尔雅可比迭代运动（TASK_FLOW）
        :param target_pos: 目标位姿 [x,y,z,qw,qx,qy,qz]
        :param line_theta_weight: 位置与姿态的权重（0~1）
        :param accuracy: 收敛精度
        :param sync: 是否同步等待
        :param tool: 工具号
        """
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
        if sync and res["recv"] == "Task_Recieve":
            self.__wait_task(res["task_key"])
        return res

    def move_joint_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True):
        """关节轨迹运动（待实现）"""
        # 当前为占位，后续可根据需要实现
        raise NotImplementedError("move_joint_traj not implemented yet")

    def move_pose_traj(self, target_traj, gripper_pos=None, stamps=None, is_sync=True):
        """位姿轨迹运动（待实现）"""
        raise NotImplementedError("move_pose_traj not implemented yet")

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

    def trajectory_recorder(self, name):
        """复现指定名称的轨迹"""
        return self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 2,
            "name": name
        })

    def check_teach(self):
        """获取已录制的轨迹列表"""
        res = self.request({
            "command": "teachRecorder",
            "arm_index": self.arm_index,
            "task_id": 3,
            "name": ""
        })
        if res.get("recv") == "Task_Recieve" and "teach_list" in res:
            return res["teach_list"]
        return []

    # -------------------- 冗余关节力矩 --------------------
    def set_redundancy_tau(self, redundancy_tau, end_effector=None):
        """设置冗余关节力矩（用于力反馈遥操作）"""
        return self.request({
            "command": "setRedundancyTau",
            "arm_index": self.arm_index,
            "is_master": True,
            "redundancy_tau": redundancy_tau
        })

    # -------------------- 运动学 --------------------
    def inverse_kine(self, cart_pose, ref_joints, tool=0):
        """
        逆运动学求解
        :param cart_pose: 目标位姿（单个或列表）
        :param ref_joints: 参考关节角（单个或列表）
        :param tool: 工具号
        :return: 返回包含关节角的响应
        """
        if not isinstance(cart_pose[0], list):
            cart_pose = [cart_pose]
            ref_joints = [ref_joints]
        assert len(cart_pose) == len(ref_joints)
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
        :return: 位姿列表或单个位姿，失败返回 None
        """
        is_list = True
        if not isinstance(joint_pos[0], list):
            joint_pos = [joint_pos]
            is_list = False
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
        except:
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
    def request(self, req):
        event = threading.Event()
        task_key = str(uuid.uuid4())
        req["task_key"] = task_key
        self.res_pool[task_key] = {"req": req, "event": event}
        self.__send(req)
        event.wait()
        return self.res_pool.pop(task_key)["res"]

    def __send(self, msg):
        self.ws.send(json.dumps(msg))

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

        # 任务完成解析（只处理当前臂）
        if len(message["arm"]) > self.arm_index:
            arm_json = message["arm"][self.arm_index]
            if "task" in arm_json:
                task_info = arm_json["task"]
                if "last_task_key" in task_info and task_info["last_task_key"]:
                    self.ops["taskFinished"]({"task_key": task_info["last_task_key"]})
                if not task_info.get("exe_flag", True):
                    self.task_event.set()

    def __cbk_taskfinish(self, message):
        task = message["task_key"]

    def __on_open(self, ws):
        self.open_ready.set()
        print("Connected successfully.")

    def __on_close(self, ws, code, close_msg):
        print("Disconnected, please check your --addr", code, close_msg)

    def __on_message(self, ws, message):
        msg = json.loads(message)
        self.last_msg = msg
        cmd = msg["command"]
        op = self.ops.get(cmd, lambda msg: self.__response_op(msg))
        op(msg)

    def __response_op(self, res):
        task_key = res.get("task_key", "")
        data = self.res_pool.get(task_key)
        if data:
            data["res"] = res
            data["event"].set()

    def __recv_loop(self):
        print("Recv loop started.")
        self.ws.run_forever()

    def __wait_task(self, task_key):
        self.task_event = threading.Event()
        self.task_event.wait()

    def __clip(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def __clip_joints(self, joints):
        lower = self.limit['limit_lower']
        upper = self.limit["limit_upper"]
        for i, v in enumerate(joints):
            joints[i] = self.__clip(v, lower[i], upper[i])
        return joints


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