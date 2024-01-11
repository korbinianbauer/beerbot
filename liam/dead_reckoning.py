import time
import math
from math import sin,cos

class OdoDeadReckoner():

    track_width = 0
    pos_history = [] #{'x': 0, 'y': 0, 'yaw': 0, 'vel': 0}
    last_left_wheel_distance = 0
    last_right_wheel_distance = 0
    initial_yaw = 0
    last_evaluate = 0
    
    
    def __init__(self, track_width, initial_yaw):
        self.track_width = track_width
        self.initial_yaw = initial_yaw
    
    def setTrackWidth(self, track_width):
        self.track_width = track_width
        
    def setInitialYaw(self, yaw):
        self.initial_yaw = yaw
    
    def getPosHistory(self):
        return self.pos_history
        
    
    
    def evaluate(self, left_wheel_distance, right_wheel_distance):
    
        time_now = time.time()
        dt = time_now - self.last_evaluate # ms
        self.last_evaluate = time_now
        
        # Convert wheel distances to robot angle change of delta_theta radians around a point drive_radius meters to left of the robot
        left_wheel_distance_delta = left_wheel_distance - self.last_left_wheel_distance
        right_wheel_distance_delta = right_wheel_distance - self.last_right_wheel_distance
        
        #print("left_wheel_distance_delta: {}".format(left_wheel_distance_delta))
        #print("right_wheel_distance_delta: {}".format(right_wheel_distance_delta))
        #print("self.track_width: {}".format(self.track_width))
        
        self.last_left_wheel_distance = left_wheel_distance
        self.last_right_wheel_distance = right_wheel_distance
        
        delta_theta = (right_wheel_distance_delta - left_wheel_distance_delta) / self.track_width # radians

        if (abs(delta_theta) > 0.001):
            drive_radius = 1/delta_theta * (right_wheel_distance_delta + left_wheel_distance_delta) / 2 # meter
        else:
            drive_radius = 9999


        # Calculate to resulting position change in the robot coordinate system
        if (drive_radius < 100):
            # if drive radius is < 100m, treat as a curve
            #print("curve")
            dy_bot = drive_radius * (1 - cos(delta_theta))
            dx_bot = drive_radius * sin(delta_theta)
        else:
            # if drive radius is >= 100m, treat as a straight
            #print("straight")
            dy_bot = 0
            dx_bot = (right_wheel_distance_delta + left_wheel_distance_delta) / 2
            
        #print("dx_bot: {}".format(dx_bot))
        #print("dy_bot: {}".format(dy_bot))
        
        if (len(self.pos_history) == 0):
            self.pos_history.append([0, 0, self.initial_yaw, 0])
        
        last_pos = self.pos_history[-1]
        yaw = last_pos[2]
			
        dx_world = cos(yaw) * dx_bot - sin(yaw) * dy_bot
        dy_world = sin(yaw) * dx_bot + cos(yaw) * dy_bot
        
        vel = math.sqrt(dx_world**2 + dy_world**2)/dt # m/s
        
        yaw = (yaw + delta_theta) % (2*math.pi)
        pos = [last_pos[0] + dx_world, last_pos[1] + dy_world, yaw, vel]
        
        self.pos_history.append(pos)
        
        return pos