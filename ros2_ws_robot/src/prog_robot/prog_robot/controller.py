"""

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

# from prog_robot.kinematics import inverse_kinematics

# from copy import deepcopy

class Controller(Node):
    
    def __init__(self):

        super().__init__('controller')      
        self.sub_coord = self.create_subscription(Float32MultiArray, '/controller/euler_pos', self.pose_callback, 10)
        self.sub_joint = self.create_subscription(Float32MultiArray, '/controller/joint_states', self.joint_callback, 10)
        # self.sub_gripper = self.create_subscription(Int32, '/controller/gripper', self.gripper_callback, 10)

        self.publisher_pose = self.create_publisher(Float32MultiArray, '/pose_euler_add', 10)
        self.publisher_joint = self.create_publisher(Float32MultiArray, '/pose_joint', 10)
        self.publisher_gripper = self.create_publisher(Int32, '/pose_gripper', 10)

        self.pose = Float32MultiArray()
        self.joint = Float32MultiArray()
        self.speed = 40.0

    # def gripper_callback(self, msg):
        
        
    def joint_callback(self, msg):
        self.joint.data = list(msg.data)
        print(self.joint)
        self.publisher_joint.publish(self.joint)

    def pose_callback(self, msg):
        self.pose.data = list(msg.data)
        print(self.pose)
        self.publisher_pose.publish(self.pose)
    


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()