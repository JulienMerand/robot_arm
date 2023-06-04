import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import time,math
from prog_robot.fonctions import ang2Step
from prog_robot.kinematics import inverse_kinematics

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('pub_to_arduino')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)
        self.sub_arduino = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.arduino_callback, 10)

        self.pub = Int32MultiArray()
        self.gripper_open = 90
        self.gripper_close = 180
        # self.poseJ = [[0,90,-90,0,90,0], [0,0,0,0,0,0]]
        # self.poseJ = [[32,37,83,0,59,36], [22,30,95,0,54,25], [0,24,104,1,51,4], [-33,33,91,1,56,-30], [-49,51,60,1,69,-46], [0,0,0,0,0,0]]
        self.p = 0
        self.start = True
        self.delay = 1
        self.poseX = [[350,0,45,0,180,0], [350,0,250,0,180,0], [250,0,250,0,180,0], [250,0,100,0,180,0]]
        self.poseJ = []
        for pose in self.poseX:
            self.poseJ.append(inverse_kinematics(pose[:3], pose[3:])[0])

        
    def arduino_callback(self, msg):
        ang_deg = self.poseJ[self.p]
        step = ang2Step(ang_deg, "deg")
        speed = 50
        accel = 70
        self.pub.data = step + [(int)(self.gripper_close)] + [(int)(speed)] + [(int)(accel)]

        if msg.data > 0 or self.start: 
            print(f"Je publie la suivante dans {self.delay} secondes : ")
            # if(not self.start):
                # time.sleep(self.delay)
            self.start = False
            self.publisher_.publish(self.pub)
            self.get_logger().info('Publishing: "%s"' % self.pub.data)
            if self.p + 1 != len(self.poseJ) :
                self.p += 1
            else: 
                self.p = 0


def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()