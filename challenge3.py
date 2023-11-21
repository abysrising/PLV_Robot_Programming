import rclpy  # ROS client library
import subprocess
from transforms3d.euler import quat2euler
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from enum import Enum, auto
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.st = State.TO_THE_FIRST_WALL
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
    
    
    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def odom_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        angles = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        x = position.x
        y = position.y
        
        print(position)
        #print(angles)
        if self.st == State.TO_THE_FIRST_WALL:
            self.vel(20,0)
            if y > 0.8:
                self.st = State.ROTATING
        elif self.st == State.ROTATING:
            self.vel(0,5) 
            if -3.1 > angles[0]:
                self.st = State.TO_THE_SECOND_WALL
        elif self.st == State.TO_THE_SECOND_WALL:
            self.vel(20,0)
            if x < 0.2:
                self.st = State.STOP

        elif self.st == State.STOP:
            self.vel(0,0)        
        

        

  
        

def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
