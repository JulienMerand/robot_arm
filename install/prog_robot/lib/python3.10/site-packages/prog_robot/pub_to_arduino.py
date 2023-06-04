import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import JointState
import time,math
from prog_robot.fonctions import ang2Step

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('pub_to_arduino')

        self.pub = Int32MultiArray()
        self.old = [-1,-1,-1,-1,-1,-1,-1]
        
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)

        self.sub_joint_states = self.create_subscription(JointState,'/joint_states', self.joint_states_callback, 10)
        self.sub_joint_states  # prevent unused variable warning

        self.sub_arduino = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.arduino_callback, 10)
        self.sub_arduino  # prevent unused variable warning
      

    def joint_states_callback(self, msg):
        ang_rad = [0,0,0,0,0,0]
        for i in range(6):
            ang_rad[i] = msg.position[i]

        step = ang2Step(ang_rad, "rad")
        pos_gripper = msg.position[6]*180/math.pi + 50
        speed = 50
        accel = 70
        self.pub.data = step + [(int)(pos_gripper)] + [(int)(speed)] + [(int)(accel)]
        
        
        
    def arduino_callback(self, msg):
        if msg.data > 0: 
            print("Effectué !")
        if self.pub.data != self.old:
            self.publisher_.publish(self.pub)
            self.get_logger().info('Publishing: "%s"' % self.pub.data)
            self.old = self.pub.data
        


def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()