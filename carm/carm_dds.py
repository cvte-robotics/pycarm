#!/usr/bin/python3
import time
import os
import argparse
import svar
import numpy as np
import carm

messenger = svar.load("svar_messenger").messenger

class ArmDriver:
    def __init__(self,args):
        self.args = args
        
        self.arm       = carm.Carm(args.addr)
        if args.mit:
            self.arm.set_control_mode(3)
        else:
            self.arm.set_control_mode(1)

        self.arm.set_speed_level(args.speed)

        self.pub_joint = messenger.advertise(args.joint_topic, 0)
        self.pub_end   = messenger.advertise(args.end_topic, 0)
        self.pub_joint_ik = messenger.advertise(args.joint_cmd_ik_topic, 0)
        
        self.sub_joint = messenger.subscribe(args.joint_cmd_topic, 0, lambda msg:self.joint_callback(msg))
        self.sub_end   = messenger.subscribe(args.end_cmd_topic, 0, lambda msg:self.end_callback(msg))

    def end_callback(self, msg):  
        position = np.frombuffer(msg["position"],dtype=np.float64).tolist()
        self.arm.track_pose(position)

        if self.args.enable_ik:
            ret = self.arm.invK(position, ref_joints=self.arm.joint_pos)
            if ret["recv"] != "Task_Recieve":
                print("Inverse kinematics failed:", ret)
                return 

            pos = ret["data"]["joint1"]
            self.pub_joint_ik.publish({"header": msg["header"], "position": pos})


    def joint_callback(self, msg):        
        position = np.frombuffer(msg["position"],dtype=np.float64).tolist()        
        self.arm.track_joint(position)

    def loop(self):
        while True:
            stamp   = time.time()
            sec     = int(stamp )
            nanosec = int((stamp - sec) * 1e9)
            header  = {"stamp": {"sec": sec, "nanosec": nanosec},"frame_id": "base_link"}

            end_msg = {"header": header, "position": self.arm.cart_pose + [self.arm.gripper_state["gripper_pos"],]}
            joints_msg = {"header": header, "position": self.arm.joint_pos + [self.arm.gripper_state["gripper_pos"],]}
            
            self.pub_joint.publish(joints_msg)
            self.pub_end.publish(end_msg)

            time.sleep(0.005)

def send_cmd(args):
    arm       = carm.Carm(args.addr)

    cmd = args.cmd

    if cmd == "disable":
       arm.set_servo_enable(False)
    if cmd == "remote":
       arm.set_control_mode(3)

def driver_main(args):
    print("Starting driver mode...")
        
    driver = ArmDriver(args)
    
    ros2 = svar.load(args.dds)
    transfer = ros2.Transfer({"node": "carm_driver" + args.device, 
                              "publishers":[[args.joint_topic, "sensor_msgs/msg/JointState",10],
                                            [args.end_topic, "sensor_msgs/msg/JointState",10],
                                            [args.joint_cmd_ik_topic, "sensor_msgs/msg/JointState",10],
                                            ],
                              "subscriptions":[[args.joint_cmd_topic, "sensor_msgs/msg/JointState",10],
                                               [args.end_cmd_topic, "sensor_msgs/msg/JointState",10]]})
    
    driver.loop()
    
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--addr", type=str, default="ws://localhost:8090", help="Device address, including ip and port")
    parser.add_argument("--cmd", type=str, default="", help="Send command instead of start driver, support enable,disable,remote")
    parser.add_argument("--device", type=str, default="carm", help="device name, used as topic prefix")
    parser.add_argument("--dds", type=str, default="svar_messenger_ros2", help="the dds plugin, default is ros2, options: svar_zbus, svar_lcm")
    parser.add_argument("--mit", action="store_true", help="Enable mit mode")
    parser.add_argument("--joint_topic", type=str, default="", help="the joints status topic")
    parser.add_argument("--end_topic", type=str, default="", help="the joints status topic")
    parser.add_argument("--joint_cmd_topic", type=str, default="", help="the joints cmd topic")
    parser.add_argument("--joint_cmd_ik_topic", type=str, default="", help="the joints cmd ik topic, this means both end_cmd_topic will be transformed with ik")
    parser.add_argument("--end_cmd_topic", type=str, default="", help="the end cmd topic")
    parser.add_argument("--speed", type=float, default=5.0, help="the speed level")
    
    args = parser.parse_args()
    
    if args.joint_topic == "":
        args.joint_topic = "/"+args.device + "/joints"
    if args.end_topic == "":
        args.end_topic = "/"+args.device + "/end"
    if args.joint_cmd_topic == "":
        args.joint_cmd_topic = "/"+args.device + "/joints_cmd"
    if args.end_cmd_topic == "":
        args.end_cmd_topic = "/"+args.device + "/end_cmd"
    if args.joint_cmd_ik_topic == "":
        args.joint_cmd_ik_topic = "/"+args.device + "/joints_cmd_ik"
        args.__dict__["enable_ik"] = False
    else:
        args.__dict__["enable_ik"] = True
    
    if args.cmd == "":
        driver_main(args)
    else:
        send_cmd(args)
   
    
# 测试代码
if __name__ == "__main__":
    main()
    
    
