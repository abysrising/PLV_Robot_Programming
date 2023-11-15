import rclpy  # ROS client library
import os
import subprocess
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from enum import Enum, auto
from sensor_msgs.msg import LaserScan
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
                Twist,
                'cmd_vel',
                1)

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,
                qos_profile_sensor_data)

        self.minimum_distance = 0.2
        self.starting_distance = 0
        self.lin_vel_percent = 0
        self.ang_vel_percent = 0

        # Gradual acceleration and deceleration parameters
        self.acceleration_rate = 1
        self.deceleration_rate = 1 
        self.st = State.TO_THE_FIRST_WALL

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        MAX_LIN_VEL = 0.1
        MAX_ANG_VEL = 1.82
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        print()
        if self.st == State.TO_THE_FIRST_WALL:
            self.drive_smoove(0.25, msg.ranges[0])
            if msg.ranges[0] <= 0.25:
                self.st = State.ROTATING
                self.starting_distance = 0
        elif self.st == State.ROTATING:
            self.vel(0, 20)
            if msg.ranges[-90] <= 0.17:
                self.st = State.TO_THE_SECOND_WALL
        elif self.st == State.TO_THE_SECOND_WALL:
            self.drive_smoove(0.2, msg.ranges[0])
            if msg.ranges[0] < 0.2:
                self.vel(0)
                self.st = State.STOP
        elif self.st == STOP:
            self.vel(0)
        

    def rotate_smoove(self, msg, rotate_direction):
        




        return 0


    def drive_smoove(self, stopping_distance, dist=0):
        if self.starting_distance in {0, float('inf')}:
            self.starting_distance = dist
            
        current_distance = dist
        if current_distance < stopping_distance:
            self.vel(0)
        else:
            velocity = 0
            current_distance_pc = current_distance / self.starting_distance

            if current_distance_pc >= 0.5:
                # Gradual acceleration
                velocity = min(self.lin_vel_percent + self.acceleration_rate, 100)
            elif current_distance_pc < 0.5:
                # Gradual deceleration
                velocity = max(self.lin_vel_percent - self.deceleration_rate, 5)
            
            self.vel(velocity)  
        

def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages....')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
