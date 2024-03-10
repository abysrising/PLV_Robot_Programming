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
from sensor_msgs.msg import LaserScan
import numpy as np

class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')
        
        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,
                qos_profile_sensor_data)

        self.cmd_vel_pub = self.create_publisher(
                Twist,
                'cmd_vel',
                1)
                
               

    
        self.clear_odom_topic()
        
        self.st = State.TO_THE_FIRST_WALL

        self.acceleration_rate = 0.1
        self.deceleration_rate = 0.1 
        self.ang_vel_percent = 2
        self.lin_vel_percent = 20

        self.direction = "UP"
        
        self.target_angle = 0

        self.target_distance = 0
        self.start_position = 0
        self.start_direction = 0

        self.start_adj = []
        self.start_adj_ang = 0
        self.set_adj = True
        self.matrix=[[0, 0], [0, 0]]
        self.laser_measurements=[]
        self.matrix_measured=False
        self.laser_measured = False
        self.matrix_position=[1, 0]
        self.robot_moved=False
        self.updated_position=True
        self.rotated = False
        self.driven = False
        self.rotation_counter = 0
        self.State_MOVEMENT = "None"
        self.target_distance_set = False
        self.start_position = 0
        self.angle_set = False

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

    def index_to_matrix_indices(self, idx, cols):
        row_index = idx // cols
        col_index = idx % cols
        return row_index, col_index
        
    def calculate_target_position(self):
        for i, row in enumerate(self.matrix):
            for j, col in enumerate(row):
                if self.matrix[i][j] == 1:
                    print("target_position: ", i, j)
                    return [i, j]
        
        
    def move_robot(self, d1, pos):
        target_position = self.calculate_target_position()
        movement_y = target_position[0] - self.matrix_position[0]
        movement_x = target_position[1] - self.matrix_position[1]
        print("x, y:", movement_x, movement_y)
        
        if self.updated_position==False:
            self.position=target_position
            print("target_position, self.position: ", target_position, self.matrix_position)
        if self.robot_moved==False:
            if movement_x==1:
                if self.driven == False:
                    self.rotate_smoove(-90, d1)
                    self.State_MOVEMENT="right"
                elif self.driven == True:
                    self.rotate_smoove(90, d1)
                    
                if self.rotated==True:
                    self.drive_smoove(pos, driving_distance=1)
            if movement_y==-1:
                print("moving in y")
                self.State_MOVEMENT="forward"
                if self.driven==True:
                    print("turning")
                    self.rotate_smoove(90, d1)
                self.drive_smoove(pos, driving_distance=1)
                
            if movement_x==-1:
                if self.driven == False:
                    self.rotate_smoove(90, d1)
                    self.State_MOVEMENT="right"
                elif self.driven == True:
                    self.rotate_smoove(-90, d1)
                    
                if self.rotated==True:
                    self.drive_smoove(pos, driving_distance=1)
  
                    
                        
                
            
        print(movement_x, movement_y)
        
        
    def create_matrix(self, msg):
        
        if self.matrix_measured==False and self.laser_measured==True:
            print("creating matrix")
            print("laser_measurements: ", self.laser_measurements)
            max_value = np.argmax(self.laser_measurements)
            row, col = self.matrix_position
            print("argmax: ", max_value)
            if max_value==0:                       
                self.matrix[row-1][col]=1
            elif max_value==1:         
                print("i2: ", row, col+1)              
                self.matrix[row][col+1]=1
            elif max_value==2:
                print("i2: ", row+1, col)
                self.matrix[row+1][col]=1
            elif max_value==3:
                self.matrix[row][col-1]=1
                        
            self.matrix[self.matrix_position[0]][self.matrix_position[1]] = 2
            self.matrix_measured=True
        print(self.matrix)
        print(self.laser_measurements)
        
    def scan_callback(self, msg):
        degree_checks = [0, 270, 180, 90]
        self.laser_measurements = [msg.ranges[degree_check] for degree_check in degree_checks]
        self.laser_measured=True
       
    
    def odom_callback(self, msg):
        print("matrix_position: ", self.matrix_position)
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
           
        print("original pos: ", pos)
       
        
        d1 = d1+math.degrees(self.start_adj_ang)
        d1 = d1 + 360 if d1 < 0 else d1

         
        
        self.create_matrix(msg)
        if self.matrix_position==[0, 1]:
            self.vel(0, 0)
            pass
        elif self.laser_measured==True:
            self.move_robot(d1, pos)
            
        print("\n")
        

    """
    def rotate_smoove(self,target_angle, current_angle):
        print("---------------", self.target_angle)
        if self.angle_set == False:
            self.target_angle=current_angle-91
            if self.target_angle<=0:
                self.target_angle = self.target_angle+360
            self.angle_set = True
                
                
        print("target angle: ", self.target_angle)
        print("current angle: ", current_angle)
        
        
        if current_angle < self.target_angle:
            
            self.vel(0, 0)
            self.target_angle = 0
            self.ang_vel_percent = 2
            self.angle_set=False
            return False
            self.rotated = True
            self.start_adj_ang += math.radians(-target_angle)
            self.vel(0, 0)
            self.target_angle = 0
            self.ang_vel_percent = 2
            if self.driven == True:
                # self.robot_moved=True
                self.driven = False
                self.matrix_measured=False
                print("==============")
                if self.State_MOVEMENT=="right":
                    self.matrix_position[1] += 1
                if self.State_MOVEMENT=="forward":
                    self.matrix_position[0] -= 1
    
        else:
          
            print("rotating")
            self.vel(0, -15)

    """
    def rotate_smoove(self, target_angle, current_angle):
        if self.rotated == False:
        #print("target angle: ", target_angle)
            error = math.fmod(target_angle - current_angle, 360)
            if math.isclose(error, 0, abs_tol=2):
                self.rotated = True
                self.start_adj_ang += math.radians(-target_angle)
                self.vel(0, 0)
                self.target_angle = 0
                self.ang_vel_percent = 2
                if self.driven == True:
                    # self.robot_moved=True
                    self.driven = False
                    self.matrix_measured=False
                    print("==============")
                    if self.State_MOVEMENT=="right":
                        self.matrix_position[1] += 1
                    if self.State_MOVEMENT=="forward":
                        self.matrix_position[0] -= 1
            else:
                if error > 180:
            	    rot_dir = 1
                else:
                    rot_dir = -1
                self.vel(0, rot_dir * 15)

    
    def check_drive_goal(self, start_position, current_position):
        print("start position: ", start_position)
        print("current position: ", current_position)
        if current_position[0] != start_position[0]:
            current_distance = math.sqrt(abs(current_position[0]-start_position[0])**2+abs(current_position[1]-start_position[1])**2)
        else:
            current_distance=0
        print("current_distance", current_distance)
        if current_distance >= 1:
            self.drive_goal_checked = True
            self.driven == True
            return True
        
   

    def drive_smoove(self, position, driving_distance = 1):
    
        print("position: ", position)
        if self.target_distance == 0 and self.target_distance_set==False:
            self.start_position = position
            self.target_distance_set = True
            self.target_distance = position[0]+driving_distance
        print("target position: ", self.target_distance)
           
        current_position = position[0]
        print("current position: ", current_position)
        print(current_position)
        print(self.target_distance)
        
        if self.check_drive_goal(self.start_position, position) == True or self.driven==True:
            
            self.vel(0, 0)
            self.target_distance = 0
            self.ang_vel_percent = 0
            self.target_distance = 0
            self.updated_position==False
            self.driven = True
            self.rotated = False
            self.rotation_counter += 1
            self.target_distance_set = False
            
        else:
            """
            velocity = 0
            current_distance_pc =  abs((current_position - self.start_position) / (self.target_distance - self.start_position))

            if current_distance_pc <= 0.5:
                # Gradual acceleration
                self.lin_vel_percent = min(self.lin_vel_percent + self.acceleration_rate, 100)
            elif current_distance_pc > 0.5:
                # Gradual deceleration
                self.lin_vel_percent = max(self.lin_vel_percent - self.deceleration_rate, 35)
            """
            self.vel(30, 0)  
        
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


