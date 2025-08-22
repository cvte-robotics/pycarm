#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import time
import argparse

from std_msgs.msg import String, Bool, Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Int8
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import AddTwoInts
import carm
import threading
import numpy as np

class ArmDriver(Node):
    def __init__(self,args):
        super().__init__('carm_'+args.device)
        self.args = args
        
        self.arm       = carm.Carm(args.addr)
        if args.mit:
            self.arm.set_control_mode(2)
        else:
            self.arm.set_control_mode(1)

        
        print("version:",self.arm.version)
        print("limits:", self.arm.limit)
        print("state:", self.arm.state)

        self.pub_joint = self.create_publisher(JointState, args.joint_topic, 10)
        self.pub_end   = self.create_publisher(JointState, args.end_topic, 10)
        
        self.sub_joint = self.create_subscription(JointState, args.joint_cmd_topic, self.joint_callback, 10)
        self.sub_end   = self.create_subscription(JointState, args.end_cmd_topic, self.end_callback, 10)

        self.worker = threading.Thread(target=self.loop).start()

    def end_callback(self, msg):  
        self.arm.track_pose(list(msg.position))

    def joint_callback(self, msg):     
        self.arm.track_joint(list(msg.position))

    def loop(self):
        while True:
            joint_msg = JointState()          # list of string
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.position = self.arm.joint_pos
            joint_msg.velocity = self.arm.joint_vel
            joint_msg.effort   = self.arm.joint_tau
            joint_msg.position.append(self.arm.gripper_state["gripper_pos"])
            self.pub_joint.publish(joint_msg)
            
            
            end_msg = JointState()          # list of string
            end_msg.header.stamp = self.get_clock().now().to_msg()
            end_msg.position = np.array(self.arm.cart_pose).tolist()
            end_msg.position.append(self.arm.gripper_state["gripper_pos"])
            self.pub_end.publish(end_msg)

            time.sleep(0.005)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--addr", type=str, default="ws://localhost:8090", help="Device address, including ip and port")
    parser.add_argument("--device", type=str, default="carm", help="device name, used as topic prefix")
    parser.add_argument("--dds", type=str, default="svar_messenger_ros2", help="the dds plugin, default is ros2, options: svar_zbus, svar_lcm")
    parser.add_argument("--mit", action="store_true", help="Enable mit mode")
    parser.add_argument("--joint_topic", type=str, default="", help="the joints status topic")
    parser.add_argument("--end_topic", type=str, default="", help="the joints status topic")
    parser.add_argument("--joint_cmd_topic", type=str, default="", help="the joints cmd topic")
    parser.add_argument("--end_cmd_topic", type=str, default="", help="the end cmd topic")
    
    args = parser.parse_args()
    
    if args.joint_topic == "":
        args.joint_topic = "/"+args.device + "/joints"
    if args.end_topic == "":
        args.end_topic = "/"+args.device + "/end"
    if args.joint_cmd_topic == "":
        args.joint_cmd_topic = "/"+args.device + "/joints_cmd"
    if args.end_cmd_topic == "":
        args.end_cmd_topic = "/"+args.device + "/end_cmd"
    
    
    rclpy.init(args=None)
    node = ArmDriver(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 测试代码
if __name__ == "__main__":
    main()
    
