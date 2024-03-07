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
import signal

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

        self.target_distance = 0
        self.start_position = 0
        self.start_direction = 0

        self.start_adj = []
        self.start_adj_ang = 0
        self.set_adj = True
     
		

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


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        angles_radian = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        angles_degree = [math.degrees(angle) for angle in angles_radian]
        x = position.x
        y = position.y
        pos = [x, y]
        
        d1 = angles_degree[2] + 360 if angles_degree[2] < 0 else angles_degree[2] 
        
        if self.set_adj == True:
            self.start_adj = [-x, -y]
            #print(self.start_adj)
            self.start_adj_ang = -angles_radian[2] 
            #print(self.start_adj_ang)
            self.set_adj = False
           
        x,y = self.translate(pos, self.start_adj)
        pos = [x, y]
        x,y = self.rotate(pos, self.start_adj_ang)
        x+=0.5
        y+=0.5
        pos = [x, y]
       
        
        d1 = d1+math.degrees(self.start_adj_ang)
        d1 = d1 + 360 if d1 < 0 else d1
        
        print(f"{pos=}\n{d1=}")
    
      
 

        if self.st == State.TO_THE_FIRST_WALL:
            self.drive_smoove(pos, State.ROTATING, 0.25)
        elif self.st == State.ROTATING:
            self.rotate_smoove(-90, d1, State.TO_THE_SECOND_WALL)
            	
        elif self.st == State.TO_THE_SECOND_WALL:
            self.drive_smoove(pos, State.STOP, 1)

        elif self.st == State.STOP:
            self.vel(0, 0)
        

    def rotate_smoove(self, target_angle, current_angle, next_state):
        #print("target angle: ", target_angle)
        error = math.fmod(target_angle - current_angle, 360)
        if math.isclose(error, 0, abs_tol=1):
            
            self.start_adj_ang += math.radians(-target_angle)
            self.vel(0, 0)
            self.st = next_state
            self.target_angle = 0
            self.ang_vel_percent = 2
    
        else:
            if error > 180:
            	rot_dir = 1
            else:
                rot_dir = -1
            self.vel(0, rot_dir * 15)


    def check_drive_goal(self, position, distance):
        print(position, distance)
        if position > distance:
            return True
        
   

    def drive_smoove(self, position, next_state, driving_distance = 0.15):
        if self.target_distance == 0:
            print(position[0])
            self.target_distance = position[0]+driving_distance

            
        current_position = position[0]
        
        
        if self.check_drive_goal(current_position, self.target_distance) == True:
            
            self.vel(0, 0)
            self.st = next_state
            self.target_distance = 0
            self.ang_vel_percent = 0
            self.target_distance = 0

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
    print("Waiting for messages...")

    def stop_robot(sig, frame):
        tb3.vel(0, 0)
        tb3.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(tb3)


if __name__ == "__main__":
    main()


