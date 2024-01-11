import sys
from math import sin, cos, fmod

PI = 3.14159265
bot_wheel_distance = 0.5948
wheel_circum_corr_factor = 0.971
IMU_cog_distance = 0.48

def eval_IMU(loglist):
    print("Evaluating IMU-ACC + IMU-Yaw:")
    start_yaw = loglist[0]["IMU_yaw"]
    last_time = loglist[0]["Time"]
    start_pos = [last_time,0,0,start_yaw] # time in ms, x, y, heading
    print("start_pos: {}".format(start_pos))
    
    positions = [start_pos]
    vel_x = 0
    
    
    for entry in loglist[1:]:
        orientation = positions[-1][3]
        t = entry["Time"]
        dt = (t - last_time)/1000.0 # in seconds
        last_time = t
        print("dt:")
        print(dt)
        acc_x = -entry["IMU_Acc_x"] # measuring negative acc in x direction means vehicle is accelerating forwards
        print("ACC raw:")
        print(acc_x)
        
        # compensate for acceleration due to rotation, since IMU is not at center of wheelbase
        delta_yaw = entry["IMU_yaw"] - orientation
        if (delta_yaw > PI):
            delta_yaw -= PI
         
        if (delta_yaw < -PI):
            delta_yaw += PI
           
        print("delta_yaw:")
        print(delta_yaw)
        
        w = delta_yaw/dt
        acc_x_comp = w*w*IMU_cog_distance
        acc_x -= acc_x_comp
        print("ACC compensated:")
        print(acc_x)
        
        
        vel_x += acc_x * dt
        
        print("vel_x:")
        print(vel_x)
        
        dx_bot = vel_x * dt
        
        # Transform position change to world coordinate system
        angle = (2*PI)- orientation
            
        dx_world = cos(angle) * dx_bot
        dy_world = sin(angle) * dx_bot
        
        print("dx_world:")
        print(dx_world)
        print("dy_world:")
        print(dy_world)
        
        pos = [positions[-1][1] + dx_world, positions[-1][2] + dy_world]
        if (orientation < 0):
            orientation += 2*PI;

        positions.append([t, pos[0], pos[1], entry["IMU_yaw"]])
            
    print("Positions reconstructed from ACC + MAG-Heading:")
    for pos in positions:
        print(pos)
    print()
       

def eval_odo_with_mag(loglist):
    print("Evaluating Odometrie and IMU-Yaw:")
    start_yaw = loglist[0]["IMU_yaw"]
    last_time = loglist[0]["Time"]
    start_pos = [last_time,0,0,start_yaw]
    print("start_pos: {}".format(start_pos))
    
    positions = [start_pos]
    old_wheel_pos = [0,0] # [Links, Rechts] in metern
    
    for entry in loglist[1:]:
        t = entry["Time"]
        orientation = positions[-1][3]

        # Im Rollstuhlmodus sind links und rechts vertauscht sowie die Vorzeichen der Radstrecke negativ
        wheel_pos = [-int(entry['Right_wheel_dist'])/1000.0, -int(entry['Left_wheel_dist'])/1000.0]
        
        left_wheel_distance = wheel_pos[0] - old_wheel_pos[0]
        right_wheel_distance = wheel_pos[1] - old_wheel_pos[1]
        old_wheel_pos = wheel_pos
        
        distance = (left_wheel_distance + right_wheel_distance)/2.0
        
        #print("distance delta:")
        #print(distance)
        
        # Transform position change to world coordinate system
        angle = (2*PI)- orientation
			
        dx_world = cos(angle) * distance
        dy_world = sin(angle) * distance
        
        odo_pos = [positions[-1][1] + dx_world, positions[-1][2] + dy_world]
        if (orientation < 0):
            orientation += 2*PI;

        positions.append([t, odo_pos[0], odo_pos[1], entry["IMU_yaw"]])

    print("Positions reconstructed from Odo + MAG-Heading:")
    for pos in positions:
        print(pos)
    print()
        
        

def eval_odo(loglist):
    pathfile = open(logfile_path + "_path_odo.txt","w")
    print("Evaluating Odometrie data:")
    start_yaw = compassHeadingToYaw(loglist[0]["IMU_yaw"])
    start_time = loglist[0]["Time"]
    start_pos = [0,0,0,start_yaw]
    print("start_pos: {}".format(start_pos))

    positions = [start_pos]
    old_wheel_pos = [0,0] # [Links, Rechts] in metern

    for entry in loglist[1:]:
        t = entry["Time"] - start_time
        odo_orientation = positions[-1][3]

        # Im Rollstuhlmodus sind links und rechts vertauscht sowie die Vorzeichen der Radstrecke negativ
        wheel_pos = [-int(entry['Right_wheel_dist'])/1000.0*wheel_circum_corr_factor, -int(entry['Left_wheel_dist'])/1000.0*wheel_circum_corr_factor]
        
        left_wheel_distance = wheel_pos[0] - old_wheel_pos[0]
        right_wheel_distance = wheel_pos[1] - old_wheel_pos[1]
        old_wheel_pos = wheel_pos

        #print("distance delta:")
        #print(left_wheel_distance, right_wheel_distance)

        # Convert wheel distances to robot angle change of delta_theta radians around a point drive_radius meters to left of the robot
        delta_theta = (right_wheel_distance - left_wheel_distance) / bot_wheel_distance # radians

        if (delta_theta != 0):
            drive_radius = 1/delta_theta * (right_wheel_distance + left_wheel_distance) / 2 # meter
        else:
            drive_radius = 9999


        # Calculate to resulting position change in the robot coordinate system
        if (drive_radius < 100):
            dy_bot = drive_radius * (1 - cos(delta_theta))
            dx_bot = drive_radius * sin(delta_theta)
        else:
            dy_bot = 0
            dx_bot = (right_wheel_distance + left_wheel_distance) / 2
			
        dx_world = cos(odo_orientation) * dx_bot - sin(odo_orientation) * dy_bot
        dy_world = sin(odo_orientation) * dx_bot + cos(odo_orientation) * dy_bot
        
        odo_pos = [positions[-1][1] + dx_world, positions[-1][2] + dy_world]
        odo_orientation = (odo_orientation + delta_theta) % (2*PI)
        if (odo_orientation < 0):
            odo_orientation += 2*PI;

        positions.append([t, odo_pos[0], odo_pos[1], odo_orientation])
        pathfile.write(str(odo_pos[0]) + "," + str(odo_pos[1]) + ",0.1\n")

    pathfile.close()
    print("Positions reconstructed from Odo:")
    for pos in positions:
        print(pos)
    print()
    
    
    
def compassHeadingToYaw(heading):
    # Convert from Compass-Heading (North=0, clockwise) to YAW (East=0, counterclockwise)
    PI = 3.141592653
    yaw = (2*PI - heading) + PI/2
    yaw = fmod(yaw, 2*PI)
    return yaw

logfile_path = sys.argv[1]
log= open(logfile_path, "r").readlines()
loglist = list()

for entry in log:
    try:
        d = eval(entry)
        loglist.append(d)
    except:
        pass


        
print("({} Entries)".format(len(loglist)))
eval_odo(loglist)
#eval_odo_with_mag(loglist)
#eval_IMU(loglist)


