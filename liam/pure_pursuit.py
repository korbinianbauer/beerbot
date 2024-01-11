import math
import numpy as np
import sys
import arduino_utils
import time

class PurePursuitController:

    path = []
    look_ahead_distance = 0
    track_width = 0
    max_velocity = 0
    max_acceleration = 0
    last_wheels_velocities = [0, 0]
    last_evaluation = 0
    current_pos = None
    
    def __init__(self, track_width, look_ahead_distance, max_velocity, max_acceleration):
        self.track_width = track_width
        self.look_ahead_distance = look_ahead_distance
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        
    def getPathEnd(self):
        return self.path[-1]
        
    def getPath(self):
        return self.path
    
    def getDistanceToTarget(self):
        if self.current_pos == None:
            # return a really big number so no one assumes we're finished
            return 1000000000000.0
        current_pos = self.current_pos
        target = self.getPathEnd()
        distance = math.sqrt((target['x'] - current_pos['x'])**2 + (target['y'] - current_pos['y'])**2)
        
        return distance
    
    def setPath(self, raw_path):
        self.path = []
        for waypoint in raw_path:
            # don't use waypoints closer than 1cm to the previous one
            if (len(self.path) == 0):
                self.path.append(waypoint)
                
            elif ((abs(waypoint['x'] - self.path[-1]['x']) + abs(waypoint['y'] - self.path[-1]['y'])) >= 0.01):
                self.path.append(waypoint)
    
        
        print("Path before cleaning: " + str(len(raw_path)) + " waypoints. After cleaning: " + str(len(self.path)) +" waypoints")

    def setTrackWidth(self, width):
        self.track_width = width
    
    def setLookAheadDistance(self, distance):
        self.look_ahead_distance = distance
        
    def setMaxVelocity(self, velocity):
        self.max_velocity = velocity
    
    def setMaxAcceleration(self, acceleration):
        self.max_acceleration = acceleration
        
    
    def evaluate(self, current_pos, current_angle):
    
        time_now = time.time()
        if (self.last_evaluation == 0):
            self.last_evaluation = time_now
            
        dt = time_now - self.last_evaluation
        self.last_evaluation = time_now
        
        self.current_pos = current_pos
        
        
        
        # remove waypoints up to closest waypoint
        
        closest_waypoint = self.path[self.closest(self.path, current_pos)]
        
        curr = self.path[0]
        while curr != closest_waypoint:
            del self.path[0]
            curr = self.path[0]
            
        # replace closest waypoint with current lookahead point
        #self.path[0] = closest_waypoint
            
        #print("Remaining path waypoint count: {}".format(len(self.path)))
        
        look_ahead_point = self.lookahead(self.path, current_pos, self.look_ahead_distance)
        closest_waypoint = self.path[self.closest(self.path, current_pos)]
        curv, angle_to_lookahead = self.curvature(self.path, current_pos, current_angle, look_ahead_point, self.look_ahead_distance)
        #print("curv: {}".format(curv))
        velocity = min(abs(closest_waypoint['vel']) + 0.05, self.max_velocity)
        velocity *= np.sign(closest_waypoint['vel'])
        #print(closest_waypoint['vel'])
        
        wheels_velocities = self.turn(curv, velocity, self.track_width) # links +, rechts - ???
        #print(wheels_velocities)
        
        #print("dt: {}".format(dt))

        for i, w in enumerate(wheels_velocities):
            wheels_velocities[i] = self.last_wheels_velocities[i] + min(self.max_acceleration*dt, max(-self.max_acceleration*dt, w-self.last_wheels_velocities[i]))

        #print("(vl, vr) = " + str([round(x, 2) for x in wheels_velocities]) + " Lookahead point: " + str([round(x, 2) for x in look_ahead_point]))
        #print("Current pos (x, y, angle) = {}, {}, {}".format(str(round(current_pos['x'], 3)), str(round(current_pos['y'], 3)), str(round(current_angle, 3))) + " Lookahead point: " + str([round(x, 2) for x in look_ahead_point]))
        #print("Recommended speeds: (vl, vr) = " + str([round(x, 2) for x in wheels_velocities]))
        #print("")
        
        self.last_wheels_velocities = wheels_velocities
        
        return wheels_velocities

        
    def closest(self, path, pos):
    
        shortpath = path[:100]

        mindist = (0, math.sqrt((shortpath[0]['x'] - pos['x']) ** 2 + (shortpath[0]['y'] - pos['y']) ** 2))
        for i, p in enumerate(shortpath):
            dist = math.sqrt((p['x']-pos['x'])**2 + (p['y']-pos['y'])**2)
            if dist < mindist[1]:
                mindist = (i, dist)

        return mindist[0]
        
        
    def lookahead(self, path, pos, lookahead_dist):
    
    
        shortpath = path[:100]
        for i, p in enumerate(reversed(shortpath[:-1])):
            i_ = len(path)-2 - i
            d = (path[i_+1]['x']-p['x'], path[i_+1]['y']-p['y'])
            f = (p['x']-pos['x'], p['y']-pos['y'])

            a = sum(j**2 for j in d)
            b = 2*sum(j*k for j,k in zip(d,f))
            c = sum(j**2 for j in f) - lookahead_dist**2
            disc = b**2 - 4*a*c
            if disc >= 0:
                disc = math.sqrt(disc)
                t1 = (-b + disc)/(2*a)
                t2 = (-b - disc)/(2*a)
                if 0<=t1<=1:
                    t = t1
                    return p['x']+t*d[0], p['y']+t*d[1]
                if 0<=t2<=1:
                    t = t2
                    return p['x']+t*d[0], p['y']+t*d[1]
        return [path[self.closest(path, pos)]['x'], path[self.closest(path, pos)]['y']]
        
        
    def curvature(self, path, pos, angle, lookahead_point, lookahead_dist):
    
        vector_RL = [lookahead_point[0]-pos['x'], lookahead_point[1]-pos['y']]
        vector_RB = [math.cos(angle), math.sin(angle)]
        
        length_RL = math.sqrt((vector_RL[0])**2 + (vector_RL[1])**2)
        length_RB = math.sqrt((vector_RB[0])**2 + (vector_RB[1])**2)
        
        
        
        dot_product = vector_RL[0] * vector_RB[0] + vector_RL[1] * vector_RB[1]
        cross_product = vector_RB[1] * vector_RL[0] - vector_RB[0] * vector_RL[1]
        
        sin_angle = cross_product / (length_RL * length_RB)
        cos_angle = dot_product / (length_RL * length_RB)
        
        angle_to_lookahead = math.atan2(sin_angle, cos_angle)
        
        print("angle_to_lookahead: {}".format(angle_to_lookahead/math.pi*180))
    
        side = np.sign(math.sin(angle)*(lookahead_point[0]-pos['x']) - math.cos(angle)*(lookahead_point[1]-pos['y']))
        
        if (side == 0): side = 1 # handle edge case
        
        if (abs(angle_to_lookahead) > math.pi/2):
            side *= 1000.0
        
        #print("side: {}".format(side))
        a = -math.tan(angle)
        c = math.tan(angle)*pos['x'] - pos['y']
        x = abs(a*lookahead_point[0] + lookahead_point[1] + c) / math.sqrt(a**2 + 1)
        
        if (x == 0): x = 0.001
        
        curv = side * (2*x/(lookahead_dist**2))
        
        #print("x: {}".format(x))
        
        return curv, angle_to_lookahead
        
        
    def turn(self, curv, vel, trackwidth):
    
    
        if (abs(curv) > 100): # treat as rotation in-place
            #print("rotate")
        
            omega_max = 0.5
    
            vel_left = np.sign(curv)*omega_max*trackwidth/2
            vel_right = -np.sign(curv)*omega_max*trackwidth/2
            
            # limit wheel speed
            vel_factor = max(1, max([abs(vel_left), abs(vel_right)])/vel)
            vel_left /= vel_factor
            vel_right /= vel_factor
            #print("Nach wheel speed limit: {}, {}".format(vel_left, vel_right))
            
            return  [vel_left, vel_right]
        
        # else treat as straight/curve
        #print("straight/curve")
    
        '''
        # limit angular speed
        print("vel before ang speed limit: {}".format(vel))
        omega = (vel_right - vel_left) / trackwidth
        print("omega before ang speed limit: {}".format(omega))
        omega_limited = min(max(-0.5, omega), 0.5)
        vel = vel * (omega_limited / omega)
        print("vel after ang speed limit: {}".format(vel))
        '''
        
        #print("Vor wheel speed limit: {}, {}".format(vel_left, vel_right))
        
        # recalculate wheel velocities
        vel_left = vel*(2+curv*trackwidth)/2
        vel_right = vel*(2-curv*trackwidth)/2
        
        # limit wheel speed
        vel_factor = max(1, max([abs(vel_left), abs(vel_right)])/vel)
        vel_left /= vel_factor
        vel_right /= vel_factor
        #print("Nach wheel speed limit: {}, {}".format(vel_left, vel_right))
        
    
        return  [vel_left, vel_right]

        
if __name__ == "__main__":
    
    pathfile_path = sys.argv[1]
    pathlines = open(pathfile_path, "r").readlines()
    path = []
    waypoint = [0,0]
    last_waypoint = waypoint

    for line in pathlines:
        waypoint = [float(x) for x in line.split(",")]
        if ((abs(waypoint[0] - last_waypoint[0]) + abs(waypoint[1] - last_waypoint[1])) > 0.01):
            path.append(waypoint)
        
        last_waypoint = waypoint

    print(path)
        


    #if __name__ == "__main__":
      
    #path = [[0,0,0.01],
    #        [0,0.2,0.5],
    #        [0,0.8,0.5],
    #        [0,1.0,0.01],
    #        [0,1.01,0]]

    width = 0.5924
    max_vel_change = 0.5
    lookahead_dist = 0.1

    pos = (0,0)
    angle = 0
    wheels = [0,0]

    dt=0.100

    verbindung = arduino_utils.get_verbindung()
    arduino_utils.reset_arduino(verbindung)
    arduino_utils.wait_for_arduino(verbindung)

    while closest(path, pos) != len(path)-1:

        look = lookahead(path, pos, lookahead_dist)
        close = closest(path, pos)
        curv = curvature(path, pos, angle, look, lookahead_dist)
        vel = path[close][2]
        last_wheels = wheels
        wheels = turn(curv, vel, width)

        for i, w in enumerate(wheels):
            wheels[i] = last_wheels[i] + min(max_vel_change*dt, max(-max_vel_change*dt, w-last_wheels[i]))

        arduino_utils.set_wheel_speed(verbindung, wheels[0], wheels[1])
        
        pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
        angle += math.atan((wheels[0]-wheels[1])/width*dt)
        print("(vl, vr) = " + str([round(x, 2) for x in wheels]) + " -> (x,y),angle = " + str([round(x, 3) for x in pos]) + "," + str(round(angle, 2)) + " Lookahead point: " + str([round(x, 2) for x in look]))
