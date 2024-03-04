import rclpy
import subprocess
import math
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
                Twist,
                'cmd_vel',
                1)

    
        self.clear_odom_topic()
        self.st = State.TO_THE_FIRST_WALL

        self.acceleration_rate = 0.1
        self.deceleration_rate = 0.1 
        self.ang_vel_percent = 2
        self.lin_vel_percent = 5

        self.direction = "UP"
        
        self.target_angle = 0
        self.starting_angle = 0

        self.target_distance = 0
        self.start_position = 0
        self.start_direction = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        MAX_LIN_VEL = 0.26
        MAX_ANG_VEL = 1.82

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def clear_odom_topic(self):
        
        self.create_subscription(
            Odometry,
            'odom',
            self.dummy_callback,
            qos_profile_sensor_data)

    def dummy_callback(self, msg):
        
        pass

    def start_odom_subscription(self):
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        angles_radian = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        angles_degree = [math.degrees(angle) for angle in angles_radian]
        x = position.x
        y = position.y
        pos = [x, y]
        d1 = angles_degree[2] + 360 if angles_degree[2] < 0 else angles_degree[2]
        
        

        if 89 <= d1 <= 91:
            self.direction = "UP"
        elif 179 <= d1 <= 181:
            self.direction = "LEFT"
        elif 269 <= d1 <= 271:
            self.direction = "DOWN"
        elif 359 <= d1 <= 360 or 0 <= d1 <= 1:
            self.direction = "RIGHT"


        if self.st == State.TO_THE_FIRST_WALL:
            self.drive_smoove("UP", pos, State.ROTATING)

        elif self.st == State.ROTATING:
            self.rotate_smoove("LEFT", d1, State.TO_THE_SECOND_WALL)
            
        elif self.st == State.TO_THE_SECOND_WALL:
            self.drive_smoove("LEFT", pos, State.STOP)

        elif self.st == State.STOP:
            self.vel(0, 0)

    def get_angular_direction(self, target_direction):
        if target_direction == "UP":
            return 90
        elif target_direction == "LEFT":
            return 180
        elif target_direction == "DOWN":
            return 270
        elif target_direction == "RIGHT":
            return 360

    def rotate_smoove(self,target_direction, current_angle, next_state):
        
        if self.target_angle == 0:
            self.target_angle = self.get_angular_direction(target_direction)
            self.starting_angle = current_angle
        
        if self.direction == target_direction:
            self.vel(0, 0)
            self.st = next_state
            self.starting_angle = 0
            self.target_angle = 0
            self.ang_vel_percent = 2
    
        else:
            current_angle_pc = (current_angle-self.starting_angle+0.1) / (self.target_angle-self.starting_angle)
            
            if current_angle_pc > 0:
                if current_angle_pc <= 0.5:
                    self.ang_vel_percent = min(self.ang_vel_percent + self.acceleration_rate, 100)
                elif current_angle_pc > 0.5:
                    self.ang_vel_percent = max(self.ang_vel_percent - self.deceleration_rate, 5)

                self.vel(0, self.ang_vel_percent)

            elif current_angle_pc < 0:
                current_angle_pc = current_angle_pc *(-1)
                if current_angle_pc <= 0.5:
                    self.ang_vel_percent = min(self.ang_vel_percent + self.acceleration_rate, 100)
                elif current_angle_pc > 0.5:
                    self.ang_vel_percent = max(self.ang_vel_percent - self.deceleration_rate, 5)

                self.vel(0, -self.ang_vel_percent)


    def get_direction(self, target_direction, max_distance):

        if target_direction == "UP":
            return 1-max_distance, 1
        elif target_direction == "LEFT":
            return 0+max_distance, 0
        elif target_direction == "DOWN":
            return 0+max_distance, 1 
        elif target_direction == "RIGHT":
            return 1-max_distance, 0 

    def check_drive_goal(self, position, distance, target_direction):

        if target_direction == "UP":
            return position > distance
        elif target_direction == "LEFT":
            return position < distance
        elif target_direction == "DOWN":
            return position < distance
        elif target_direction == "RIGHT":
            return position > distance

    def drive_smoove(self, target_direction, position, next_state, max_distance = 0.2):


        if self.target_distance == 0:
            self.target_distance, self.start_direction = self.get_direction(target_direction, max_distance)
            self.start_position = position[self.start_direction]

        current_position = position[self.start_direction]
        if self.check_drive_goal(current_position, self.target_distance, target_direction) == True:
            self.vel(0, 0)
            self.st = next_state
            self.target_distance = 0
            self.ang_vel_percent = 0

        else:
            velocity = 0
            current_distance_pc =  abs((current_position - self.start_position) / (self.target_distance - self.start_position))

            if current_distance_pc <= 0.5:
                # Gradual acceleration
                self.lin_vel_percent = min(self.lin_vel_percent + self.acceleration_rate, 100)
            elif current_distance_pc > 0.5:
                # Gradual deceleration
                self.lin_vel_percent = max(self.lin_vel_percent - self.deceleration_rate, 5)
            
            self.vel(self.lin_vel_percent, 0)  
        
def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    tb3.start_odom_subscription()

    try:
        rclpy.spin(tb3)
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
