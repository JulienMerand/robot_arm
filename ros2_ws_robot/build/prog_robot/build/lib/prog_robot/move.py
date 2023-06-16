import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
from prog_robot.fonctions import ang2Step, cercle
from prog_robot.kinematics import inverse_kinematics

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('pub_to_arduino')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)
        # self.sub_arduino = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.arduino_callback, 10)
        self.sub_coord = self.create_subscription(Float32MultiArray, '/pose_xyz_speed', self.pose_callback, 10)
        self.sub_joint = self.create_subscription(Float32MultiArray, '/pose_theta_speed', self.joint_callback, 10)
        self.sub_gripper = self.create_subscription(Int32, '/pose_gripper', self.gripper_callback, 10)

        self.pub = Int32MultiArray()
        self.old_pub = Int32MultiArray()
        self.old_pub.data = self.pub.data
        self.pose_gripper = Int32()
        self.pose_gripper.data = 90
        self.poseJ = []
        self.p = 0
        self.start = True
        self.delay = 1

        self.speed = 15
        self.accel = 30

        self.elbow_config = "above"
        self.wrist_turned = False

    def gripper_callback(self, msg):
        if msg.data >= 90 and msg.data <= 180:
            self.pose_gripper.data = msg.data
            self.old_pub.data[7] =  self.pose_gripper.data
            self.publisher_.publish(self.old_pub)
            self.get_logger().info('Publishing: "%s"' % self.old_pub.data)
        else:
            self.get_logger().info('Error, must be between 90 (open) and 180 (close)')
        
    def joint_callback(self, msg):
        if len(msg.data) == 7:
            self.poseJ = msg.data[:6]
            self.speed = msg.data[6]
            step = ang2Step(self.poseJ, "deg")
            self.pub.data = step + [(int)(self.pose_gripper.data)] + [(int)(self.speed)] + [(int)(self.accel)]
            self.publisher_.publish(self.pub)
            self.get_logger().info('Publishing: "%s"' % self.pub.data)
            self.old_pub.data = self.pub.data
            
        else:
            self.get_logger().info('some data are missing, must be [J1, J2, J3, J4, J5, J6, gripper_pose]')

    def pose_callback(self, msg):
        if len(msg.data) == 7:
            self.poseX = msg.data[:6]
            self.speed = msg.data[6]
            IK = inverse_kinematics(self.poseX[:3], self.poseX[3:6], self.elbow_config, self.wrist_turned)
            if IK != []:
                self.poseJ = IK
                step = ang2Step(self.poseJ, "deg")
                self.pub.data = step + [(int)(self.pose_gripper.data)] + [(int)(self.speed)] + [(int)(self.accel)]
                self.publisher_.publish(self.pub)
                self.get_logger().info('Publishing: "%s"' % self.pub.data)
                self.old_pub.data = self.pub.data
            else:
                self.get_logger().info('No solutions found by the inverse kinematics')
        else:
            self.get_logger().info('some data are missing, must be [x, y, z, A, B, C, gripper_pose]')


def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()