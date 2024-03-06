import rclpy
import subprocess
import math
from transforms3d.euler import quat2euler
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from enum import Enum, auto
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import fmod, pi

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

        self.start_adj = []
        self.start_adj_ang = 0
        self.set_adj = True
        self.start_ori = "RIGHT"


    def vel(self, lin_vel_percent, ang_vel_percent=0):
        MAX_LIN_VEL = 0.5
        MAX_ANG_VEL = 3.64

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


    def rotate(self, point: tuple[float, float], angle: float):     
    	from math import cos, sin     
    	return (         
    		point[0] * cos(angle) - point[1] * sin(angle),         
    		point[1] * cos(angle) + point[0] * sin(angle),     ) 
    		
    def translate(self, point: tuple[float, float], translation: tuple[float, float]):     
    	return point[0] + translation[0], point[1] + translation[1]


    def ori(self, degree):
        if 89 <= degree <= 91:
            return "UP"
        elif 179 <= degree <= 181:
            return "LEFT"
        elif 269 <= degree <= 271:
            return "DOWN"
        elif 359 <= degree <= 360 or 0 <= degree <= 1:
            return "RIGHT"

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        angles_radian = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        angles_degree = [math.degrees(angle) for angle in angles_radian]
        x = position.x
        y = position.y
        pos = [x, y]
        d1 = angles_degree[2] + 360 if angles_degree[2] < 0 else angles_degree[2] 
        #print("position1: ", pos)
        if self.set_adj == True:
            self.start_adj = [pos[0], pos[1]]
            self.start_adj_ang = d1
            self.set_adj = False
            self.start_ori = self.ori(0)
        x,y = self.rotate(pos, self.start_adj_ang)
        pos = [x, y]
        #print("position2: ", pos)
        x,y = self.translate(pos, self.start_adj)
        pos = [-x, y]

        d1 = d1 - self.start_adj_ang
        d1 = d1 + 360 if d1 < 0 else d1
        #
        print(f"{pos=}\n{d1=}")
        #print(self.start_adj, self.start_adj_ang)
        
        self.direction = self.ori(d1)



        if self.st == State.TO_THE_FIRST_WALL:
            self.drive_smoove(self.start_ori, pos, State.ROTATING)
        elif self.st == State.ROTATING:
            self.rotate_smoove("DOWN", d1, State.TO_THE_SECOND_WALL)
        elif self.st == State.TO_THE_SECOND_WALL:
            x,y = self.rotate(pos, 270)
            pos = [x, y]
            print(pos)
            self.drive_smoove("DOWN", pos, State.STOP)

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
            
            #current_angle_pc = abs(current_angle-self.starting_angle) / abs(self.target_angle-self.starting_angle)
            #print(current_angle_pc)
            #if current_angle_pc <= 0.5:
            #    self.ang_vel_percent = min(self.ang_vel_percent + self.acceleration_rate, 100)
            #elif current_angle_pc > 0.5:
            #    self.ang_vel_percent = max(self.ang_vel_percent - self.deceleration_rate, 5)
       
            self.vel(0, -15)

    def get_direction(self, position, target_direction, driving_distance, tile_goal=0):

        if target_direction == "UP":
            return position[0]+driving_distance+tile_goal, 0
        elif target_direction == "LEFT":
            return position[0]+driving_distance+tile_goal, 0
        elif target_direction == "DOWN":
            return position[0]+driving_distance+tile_goal, 0 
        elif target_direction == "RIGHT":
            return position[0]+driving_distance+tile_goal, 0 

    def check_drive_goal(self, position, distance, target_direction):

        if target_direction == "UP":
            return position > distance
        elif target_direction == "LEFT":
            return position > distance
        elif target_direction == "DOWN":
            return position > distance
        elif target_direction == "RIGHT":
            return position > distance

    def drive_smoove(self, target_direction, position, next_state, driving_distance = 0.15):
        if self.target_distance == 0:
            self.target_distance, self.start_direction = self.get_direction(position, target_direction, driving_distance)
            if self.target_distance < 0:
                self.target_distance = 1 + self.target_distance
            print(self.target_distance)
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
