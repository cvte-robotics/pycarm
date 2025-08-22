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
            on_open   = lambda ws: self.on_open(ws),
            on_close  = lambda ws, code, close_msg: self.on_close(ws, code, close_msg),
            on_message= lambda ws, msg: self.on_message(ws, msg),
        )

        self.ops = {
            "webSendRobotState": lambda msg: self.cbk_status(msg),
            "taskFinished": lambda msg: self.cbk_taskfinish(msg),
            "onCarmError": lambda msg: print("Error:", msg)
        }
        self.res_pool  = {}
        self.task_pool = {}

        self.reader = threading.Thread(target=self.recv_loop, daemon=True).start()
        self.open_ready = threading.Event()
        self.open_ready.wait()
        self.limit = self.get_limits()["params"]

        self.set_ready()

    @property
    def version(self):
        return self.request({"command":"getArmIntrinsicProperties",
                             "arm_index":0,
                             "type":"version"})
    

    def get_limits(self):
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
        return self.request({"command":"setServoEnable",
                             "arm_index":0,
                             "enable":enable})
    
    def set_control_mode(self, mode=1):
        return self.request({"command":"setControlMode",
                             "arm_index":0,
                             "mode":mode})
    
    def set_end_effector(self, pos, tau):
        pos = self.__clip(pos, 0, 0.08)
        tau = self.__clip(tau, 0, 9)

        return self.request({"command":"setEffectorCtr",
                             "arm_index":0,
                             "pos": pos,
                             "tau": tau})
    
    def get_tool_coordinate(self, tool):
        return self.request({"command":"getCoordinate",
                             "arm_index":0,
                             "type": "tool",
                             "index": tool})

    
    def set_collision_config(self, flag = True, level = 10):
        return self.request({"command":"setCollisionConfig",
                             "arm_index":0,
                             "flag": flag,
                             "level": level})
    
    def stop(self, type=0):
        stop_id = ["SIG_ARM_PAUSE", "SIG_ARM_STOP", 
                   "SIG_ARM_DISABLE","SIG_EMERGENCY_STOP"]
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id":  stop_id[type],
                             "step_cnt":5})
    
    def stop_task(self, at_once=False):
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id": "SIG_TASK_STOP",
                             "stop_at_once": at_once})
    
    def recover(self):
        return self.request({"command":"stopSignals",
                             "arm_index":0,
                             "stop_id": "SIG_ARM_RECOVER",
                             "step_cnt": 5})
    
    def clean_carm_error(self):
        return self.request({"command":"setControllerErrorReset",
                             "arm_index":0})
    
    def set_speed_level(self, level, response_level):
        return self.request({"command":"setSpeedLevel",
                             "arm_index":0,
                             "level":level,
                             "response_level":response_level})
    
    def set_debug(self, flag):
        return self.request({"command":"setDebugMode",
                             "arm_index":0,
                             "trigger":flag})

    def get_urdf(self):
        return self.request({"command":"getArmIntrinsicProperties",
                             "arm_index":0,
                             "type":"urdf"})   
    @property
    def joint_pos(self):
        return self.state["arm"][0]["reality"]["pose"]
    
    @property
    def joint_vel(self):
        return self.state["arm"][0]["reality"]["vel"]
    
    @property
    def joint_tau(self):
        return self.state["arm"][0]["reality"]["torque"]
    
    @property
    def cart_pose(self):
        return self.state["arm"][0]["pose"]
    
    @property
    def gripper_state(self):
        return self.state["arm"][0]["gripper"]
    
    def clip_joints(self, joints):
        lower = self.limit['limit_lower']
        upper = self.limit["limit_upper"]
        for i,v in enumerate(joints):
            joints[i] = self.__clip(v, lower[i], upper[i])
        return joints
    
    def track_joint(self, pos, end_effector = -1):
        pos = self.clip_joints(pos)
        req = {"command":"trajectoryTrackingTasks",
               "task_id":"TASK_TRACKING",
               "arm_index":0,
               "point_type":{"space":0},
               "data":{"way_point": pos}}
        
        if end_effector >= 0:
            req["data"]["eeffe_point"] = end_effector

        return self.request(req)
    
    def track_pose(self, pos, end_effector = -1):
        req = {"command":"trajectoryTrackingTasks",
               "task_id":"TASK_TRACKING",
               "arm_index":0,
               "point_type":{"space":1},
               "data":{"way_point": pos}}
        
        if end_effector > 0:
            req["data"]["eeffe_point"] = end_effector

        return self.request(req)

    def move_joint(self, pos, tm=-1, sync=True, user=0, tool=0):
        pos = self.clip_joints(pos)
        
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVJ",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":0},
                             "data":{"user":user,"tool":tool, "target_pos": pos, "speed":100}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.wait_task(res["task_key"])

        return res

    def move_pose(self, pos, tm=-1, sync=True, user=0, tool=0):
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVJ",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":1},
                             "data":{"user":user,"tool":tool, "target_pos": pos, "speed":100}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.wait_task(res["task_key"])

        return res
    

    def move_line(self, pos, speed=50, sync=True, user=0, tool=0):
        res =  self.request({"command":"webRecieveTasks",
                             "task_id":"TASK_MOVL",
                             "task_level":"Task_General",
                             "arm_index":0,
                             "point_type":{"space":1},
                             "data":{"user":user,"tool":tool, "target_pos": pos, "speed":speed}}) 
        
        if sync and res["recv"]=="Task_Recieve":
            self.wait_task(res["task_key"])

        return res
        
    def wait_task(self, task_key):
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
        if not type(cart_pose[0]) is list:
            cart_pose = [cart_pose, ]
        assert(len(cart_pose) == len(ref_joints))
        return self.request({"command":"getKinematics",
                             "task_id":"inverse",
                             "arm_index":0,
                             "data":{"user":user,"tool":tool,"point_cnt":len(cart_pose)}})

    def request(self, req):
        event = threading.Event()
        task_key = str(uuid.uuid4())
        req["task_key"] = task_key
        self.res_pool[task_key] = {"req":req,"event":event}

        self.send(req)

        event.wait()
        return self.res_pool.pop(task_key)["res"]
        
    def send(self, msg):
        self.ws.send(json.dumps(msg))
        
    def cbk_status(self, message):
        self.state = message

    def cbk_taskfinish(self, message):
        task = message["task_key"]
        self.task_pool[task]["event"].set()
        
    def on_open(self, ws):
        self.open_ready.set()
        print("Connected successfully.")

    def on_close(self, ws, code, close_msg):
        print("Disconnected, please check your --addr",code, close_msg)

    def on_message(self, ws, message):
        msg = json.loads(message)
        self.last_msg = msg
        cmd = msg["command"]
        op = self.ops.get(cmd, lambda msg: self.response_op(msg))
        op(msg)

    def response_op(self, res):
        id   = res.get("task_key","")
        data = self.res_pool[id]
        data["res"] = res
        data["event"].set() # notify request thread


    def recv_loop(self):
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