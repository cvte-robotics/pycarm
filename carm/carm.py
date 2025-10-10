import websocket
import threading
import json
import uuid
import time

class Carm:
    def __init__(self, addr = "ws://100.84.147.120:8090"):
        self.state = None
        self.last_msg = None
        self.ws = websocket.WebSocketApp(
            addr,  # 测试用的公开WebSocket服务
            on_open   = lambda ws: self.__on_open(ws),
            on_close  = lambda ws, code, close_msg: self.__on_close(ws, code, close_msg),
            on_message= lambda ws, msg: self.__on_message(ws, msg),
        )

        self.ops = {
            "webSendRobotState": lambda msg: self.__cbk_status(msg),
            "taskFinished": lambda msg: self.__cbk_taskfinish(msg),
            "onCarmError": lambda msg: print("Error:", msg)
        }
        self.res_pool  = {}
        self.task_pool = {}

        self.reader = threading.Thread(target=self.__recv_loop, daemon=True).start()
        self.open_ready = threading.Event()
        self.open_ready.wait()
        self.limit = self.get_limits()["params"]

        self.set_ready()

    @property
    def version(self):
        """
        Get the version of the arm.
        Returns:
            dict: The version of the arm.
        """
        return self.request({"command":"getArmIntrinsicProperties",
                             "arm_index":0,
                             "type":"version"})
    

    def get_limits(self):
        """
        获取主要配置参数，包括关节限位、关节最大速度、加速度、加加速度等。
        Returns:
            dict: The limits of the arm.
        """
        return self.request({"command":"getJointParams",
                             "arm_index":0,
                             "type":"version"})

    def set_ready(self):
        while self.state is None:
            time.sleep(0.1)
        arm = self.state["arm"][0]
        if arm["fsm_state"] == "POSITION" or arm["fsm_state"] == "MIT":
            return True
        
        if arm["fsm_state"] == "ERROR":
            self.clean_carm_error()
        
        if arm["fsm_state"] == "IDLE":
            self.set_servo_enable(True)

        return self.set_control_mode(1)

    def set_servo_enable(self, enable=True):
        """
        Set the servo enable of the arm.
        Args:
            enable (bool): The servo enable to set.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setServoEnable",
                             "arm_index":0,
                             "enable":enable})
    
    def set_control_mode(self, mode=1):
        """
        Set the control mode of the arm.
        Args:
            mode (int): The control mode to set.
            * 0-IDLE空闲模式
            * 1-点位控制模式
            * 2-MIT控制模式
            * 3-关节拖动模式
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setControlMode",
                             "arm_index":0,
                             "mode":mode})
    
    def set_end_effector(self, pos, tau):
        """
        Set the end effector of the arm.
        Args:
            pos (float): The position of the end effector. 0-0.08m
            tau (float): The torque of the end effector. 0-9.0N.m
        Returns:
            dict: The response from the arm.
        """
        pos = self.__clip(pos, 0, 0.08)
        tau = self.__clip(tau, 0, 9)

        return self.request({"command":"setEffectorCtr",
                             "arm_index":0,
                             "pos": pos,
                             "tau": tau})
    
    def get_tool_coordinate(self, tool):
        """
        获取指定工具的坐标系（工具末端相对法兰的位姿关系）.
        Args:
            tool (int): The tool index.
        Returns:
            dict: The tool coordinate of the arm.
        """
        return self.request({"command":"getCoordinate",
                             "arm_index":0,
                             "type": "tool",
                             "index": tool})

    
    def set_collision_config(self, flag = True, level = 10):
        """
        设置碰撞检测配置.
        Args:
            flag (bool): 是否开启碰撞检测.
            level (int): 碰撞检测等级. 灵敏度等级 0-2，0最高，2最低
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setCollisionConfig",
                             "arm_index":0,
                             "flag": flag,
                             "level": level})
    
    def stop(self, type=0):
        """
        停止carm.
        Args:
            type (int): 停止类型.
            * 0-暂停
            * 1-停止
            * 2-禁用
            * 3-紧急停止
        Returns:
            dict: The response from the arm.
        """
        stop_id = ["SIG_ARM_PAUSE", "SIG_ARM_STOP", 
                   "SIG_ARM_DISABLE","SIG_EMERGENCY_STOP"]
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id":  stop_id[type],
                             "step_cnt":5})
    
    def stop_task(self, at_once=False):
        """
        停止当前任务.
        Args:
            at_once (bool): 是否立即停止任务.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id": "SIG_TASK_STOP",
                             "stop_at_once": at_once})
    
    def recover(self):
        """
        恢复carm.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id": "SIG_ARM_RECOVER",
                             "step_cnt": 5})
    
    def clean_carm_error(self):
        """
        清除carm错误.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setControllerErrorReset",
                             "arm_index":0})
    
    def set_speed_level(self, level = 5.0, response_level = 20):
        """
        Set the speed level of the arm.
        Args:
            level (float): The speed level to set. range from 0 to 10. 0 is the slowest, 10 is the fastest.
            response_level (int): The response level to set, after how many cicles to set. From 10 to 100.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setSpeedLevel",
                             "arm_index":0,
                             "level":level,
                             "response_level":response_level})
        
    def set_debug(self, flag):
        """
        设置调试模式.
        Args:
            flag (bool): 是否开启调试模式.
        Returns:
            dict: The response from the arm.
        """
        return self.request({"command":"setDebugMode",
                             "arm_index":0,
                             "trigger":flag})

    def get_urdf(self):
        """
        获取urdf模型.
        Returns:
            dict: The urdf model of the arm.
        """
        return self.request({"command":"getArmIntrinsicProperties",
                             "arm_index":0,
                             "type":"urdf"})   
    @property
    def joint_pos(self):
        """
        获取当前关节位置.
        Returns:
            list: The joint position of the arm.
        """
        return self.state["arm"][0]["reality"]["pose"]
    
    @property
    def joint_vel(self):
        """
        获取当前关节速度.
        Returns:
            list: The joint velocity of the arm.
        """
        return self.state["arm"][0]["reality"]["vel"]
    
    @property
    def joint_tau(self):
        """
        获取当前关节力矩.
        Returns:
            list: The joint torque of the arm.
        """
        return self.state["arm"][0]["reality"]["torque"]
    
    @property
    def cart_pose(self):
        """
        获取当前笛卡尔位置.
        Returns:
            list: The cartesian position of the arm.
        """
        return self.state["arm"][0]["pose"]
    
    @property
    def gripper_state(self):
        """
        获取当前夹爪状态.
        Returns:
            dict: The gripper state of the arm.
        """
        return self.state["arm"][0]["gripper"]
    
    def __clip_joints(self, joints):
        lower = self.limit['limit_lower']
        upper = self.limit["limit_upper"]
        for i,v in enumerate(joints):
            joints[i] = self.__clip(v, lower[i], upper[i])
        return joints
    
    def track_joint(self, pos, end_effector = -1):
        """
        关节轨迹跟踪.
        Args:
            pos (list): The joint position to track.
            end_effector (float, optional): The end effector width. -1 means not set.
        Returns:
            dict: The response from the arm.
        """
        pos = self.__clip_joints(pos)
        req = {"command":"trajectoryTrackingTasks",
               "task_id":"TASK_TRACKING",
               "arm_index":0,
               "point_type":{"space":0},
               "data":{"way_point": pos}}
        
        if end_effector >= 0:
            req["data"]["eeffe_point"] = end_effector

        return self.request(req)
    
    def track_pose(self, pos, end_effector = -1):
        """
        笛卡尔轨迹跟踪.
        Args:
            pos (list): The cartesian position to track.
            end_effector (float, optional): The end effector width. -1 means not set.
        Returns:
            dict: The response from the arm.
        """
        req = {"command":"trajectoryTrackingTasks",
               "task_id":"TASK_TRACKING",
               "arm_index":0,
               "point_type":{"space":1},
               "data":{"way_point": pos}}
        
        if end_effector > 0:
            req["data"]["eeffe_point"] = end_effector

        return self.request(req)

    def move_joint(self, pos, tm=-1, sync=True, user=0, tool=0):
        """
        关节到点运动
        Args:
            pos (list): The joint position to move.
            tm (float, optional): The time to move. -1 means not set.
            sync (bool, optional): Whether to wait for the movement to finish.
            user (int, optional): The user index.
            tool (int, optional): The tool index.
        Returns:
            dict: The response from the arm.
        """
        pos = self.__clip_joints(pos)
        
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVJ",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":0},
                             "data":{"user":user,"tool":tool, "target_pos": pos, "speed":100}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.__wait_task(res["task_key"])

        return res

    def move_pose(self, pos, tm=-1, sync=True, user=0, tool=0):
        """
        笛卡尔到点运动
        Args:
            pos (list): The cartesian position to move.
            tm (float, optional): The time to move. -1 means not set.
            sync (bool, optional): Whether to wait for the movement to finish.
            user (int, optional): The user index.
            tool (int, optional): The tool index.
        Returns:
            dict: The response from the arm.
        """
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVJ",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":1},
                             "data":{"user":user,"tool":tool, "target_pos": pos, "speed":100}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.__wait_task(res["task_key"])

        return res
    

    def move_line(self, pos, speed=50, sync=True, user=0, tool=0):
        """
        笛卡尔直线运动
        Args:
            pos (list): The cartesian position to move.
            speed (float, optional): The speed to move. 0 - 100%.
            sync (bool, optional): Whether to wait for the movement to finish.
            user (int, optional): The user index.
            tool (int, optional): The tool index.
        Returns:
            dict: The response from the arm.
        """
        ret = self.invK(pos, self.joint_pos, user, tool) # check IK first
        if ret["recv"] != "Task_Recieve":
            print("Inverse kinematics failed:", ret)
            return ret

        pos = ret["data"]["joint1"]
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVL",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":0},
                             "data":{"user":user,"tool":tool, "point": pos, "speed":speed}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.__wait_task(res["task_key"])

        return res
        
    def __wait_task(self, task_key):
        event = threading.Event()
        self.task_pool[task_key] = {"event":event}
        event.wait()
        self.task_pool.pop(task_key)
    
    def move_joint_traj(self, target_traj, gripper_pos = [], stamps = [], is_sync=True):
        if len(stamps) != len(target_traj): # as soon as possible
            return self.move_toppra(target_traj, gripper_pos, 100, is_sync)
        else:
            return self.move_pvt(target_traj, gripper_pos, stamps, is_sync)        
    
    def move_pose_traj(self, target_traj, gripper_pos = [], stamps = [], is_sync=True):
        if len(stamps) != len(target_traj): # as soon as possible
            return self.move_toppra(target_traj, gripper_pos, 100, is_sync)
        else:
            return self.move_pvt(target_traj, gripper_pos, stamps, is_sync)
        
    def move_toppra(self, target_traj, gripper_pos = [], speed = [], is_sync=True):
        pass # TODO

    def move_pvt(self, target_traj, gripper_pos = [], stamps = [], is_sync=True):
        pass # TODO

    def invK(self, cart_pose, ref_joints, user=0, tool=0):
        """
        逆解
        Args:
            cart_pose (list): The cartesian position to move.
            ref_joints (list): The reference joint position.
            user (int, optional): The user index.
            tool (int, optional): The tool index.
        Returns:
            dict: The response from the arm.
        """
        if not type(cart_pose[0]) is list:
            cart_pose = [cart_pose, ]
            ref_joints = [ref_joints, ]
        assert(len(cart_pose) == len(ref_joints))
        data = {"user":user,"tool":tool,"point_cnt":len(cart_pose)}
        for i in range(len(ref_joints)):
            data[f"point{i+1}"] = cart_pose[i]
            data[f"refer{i+1}"] = ref_joints[i]
        return self.request({"command":"getKinematics",
                             "task_id":"inverse",
                             "arm_index":0,
                             "data":data})

    def request(self, req):
        event = threading.Event()
        task_key = str(uuid.uuid4())
        req["task_key"] = task_key
        self.res_pool[task_key] = {"req":req,"event":event}

        self.__send(req)

        event.wait()
        return self.res_pool.pop(task_key)["res"]
        
    def __send(self, msg):
        self.ws.send(json.dumps(msg))
        
    def __cbk_status(self, message):
        self.state = message

    def __cbk_taskfinish(self, message):
        task = message["task_key"]
        self.task_pool[task]["event"].set()
        
    def __on_open(self, ws):
        self.open_ready.set()
        print("Connected successfully.")

    def __on_close(self, ws, code, close_msg):
        print("Disconnected, please check your --addr",code, close_msg)

    def __on_message(self, ws, message):
        msg = json.loads(message)
        self.last_msg = msg
        cmd = msg["command"]
        op = self.ops.get(cmd, lambda msg: self.__response_op(msg))
        op(msg)

    def __response_op(self, res):
        id   = res.get("task_key","")
        data = self.res_pool[id]
        data["res"] = res
        data["event"].set() # notify request thread


    def __recv_loop(self):
        print("Recv loop started.")
        self.ws.run_forever()

    def __clip(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

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