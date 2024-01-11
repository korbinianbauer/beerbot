import serial
import time
from math import sin, cos

bot_wheel_distance = 0.65 # distance between left and right wheel in meter

PI = 3.141592

logfile = None


start_time = time.time()

verbindung = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200, timeout=1, write_timeout=1)

verbindung.setDTR(False)
time.sleep(1)
# toss any data already received, see
# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
verbindung.flushInput()
verbindung.setDTR(True)



while (time.time() - start_time) < 10: # Wait for Magnetometer/Gyro Fusion to fix
    verbindung.readline()


start_time = time.time()
##while True:
##    print(verbindung.readline())
##    verbindung.write("Hi")





def set_logfile():
    filename = time.ctime(time.time())
    
    filename += "_Program_"
    filename += str(program_to_run+1)
    filename = filename.replace(":", "-")
    filename = filename.replace(" ", "_")
    print(filename)
    global logfile
    logfile = open("/home/pi/beerbot/log/" + filename + ".txt","w")





acc_pos = [0,0,0]
acc_vel = [0,0,0]

odo_wheel_pos = [0,0]
odo_pos = [0,0]
odo_orientation = 0
odo_vel = [0,0]
last_time = 0
time_now = 0

last_instruction_sent = 0
instruction_interval = 0.1



program1 = [[0, 0, 0],
            [10, 0, 0]]
program1_desc = "Program 1: 10s Stillstand"

program2 = [[0, -0.5, -0.5],
            [10, 0, 0]]
program2_desc = "Program 2: 10s Geradeaus bei 0.5m/s"

program3 = [[0, 0.5, -0.5],
            [10, 0, 0]]
program3_desc = "Program 2: 10s Rotieren auf der Stelle bei +-0.5m/s Radgeschwindigkeit"

program4 = [[0, -0.5, -0.5],
            [2, 0, -0.5],
            [2.5, -0.5, -0.5],
            [4.5, 0, -0.5],
            [5, -0.5, -0.5],
            [7, 0, -0.5],
            [7.5, -0.5, -0.5],
            [9.5, 0, -0.5],
            [10, 0, 0]]
program4_desc = "Program 4: 2s geradeaus bei 0.5m/s, 0.5s Drehung nach rechts bei +-0.5m/s, 4 Wiederholungen"

programs = [program1, program2, program3, program4]
program_descs = [program1_desc, program2_desc, program3_desc, program4_desc]

program_to_run = 0


set_logfile()

def log_bot():
    global verbindung
    verbindung.write("REQ")
    antwort = verbindung.readline()
    logfile.write(antwort)

def log_prog_desc():
    global verbindung
    logfile.write(time.ctime(time.time()) + "\n")
    logfile.write(program_descs[program_to_run] + "\n")


def set_wheel_speed(left_speed, right_speed):
    outstring = "CMDIN,"
    outstring += "{:05}".format(int(left_speed*1000))
    outstring += ","
    outstring += "{:05}".format(int(right_speed*1000))
    outstring += ",END\n"


    global verbindung
    verbindung.write(outstring)

def drive_by_program(program, time_t):
    
    global last_instruction_sent
    global instruction_interval
    global logfile
    if (last_instruction_sent + instruction_interval) > time_t:
        return

    if (len(program) > 1):
        if (program[0][0] <= time_t) and (program[1][0] <= time_t):
            del program[0]

    
    for instruction in reversed(program):
        start_time, left_speed, right_speed = instruction

        
        if start_time <= time_t:
            set_wheel_speed(left_speed, right_speed)
            last_instruction_sent = time_t
            log_bot()
            
            

            if (len(program) == 1):
                print("Done!")
                del program[0]
                
                logfile.close()
            return

def dict_to_vals(d):
    try:
        data = eval(d)
    except:
        print("{}: String is not a valid dict, skipping: {}".format(time.time(), antwort))
        return []

    try:
        time_now = float(data["Time"])
        Odo = [-long(data["Left_wheel_dist"]), -long(data["Right_wheel_dist"])]
        Acc = [float(data["IMU_Acc_x"]), float(data["IMU_Acc_y"]), float(data["IMU_Acc_z"])]
        Dir = [float(data["IMU_roll"]), float(data["IMU_pitch"]), float(data["IMU_yaw"])]
    except:
        #print("{}: Dictionary is not valid, skipping: {}".format(time.time(), data))
        return []
    
#    print("Data received: Time: {:08.2f}s, Odo: {}, Acc: {}, Roll-Pitch-Yaw: {}".format(time_now/1000, Odo, Acc, Dir))

    return time_now, Odo, Acc, Dir
    

try:
    
    log_prog_desc()
    
    while True:
##        antwort = verbindung.readline()
##        try:
##            time_now, Odo, Acc, Dir = dict_to_vals(antwort)
##        except:
##            continue
##
##        if (0 == last_time): # No valid time signal detected yet, don't evaluate Anything else
##            last_time = time_now
##            continue

        time_now = time.time() - start_time
        
        drive_by_program(programs[program_to_run], time_now)

        
##        dt = (time_now - last_time)/1000 #ms to s
##
##        # Estimate velocity and position based on Odometry
##        left_wheel_distance = Odo[0]/1000.0 - odo_wheel_pos[0] #mm to m
##        right_wheel_distance = Odo[1]/1000.0 - odo_wheel_pos[1] #mm to m
##        odo_wheel_pos = [Odo[0]/1000.0, Odo[1]/1000.0]
##
###        print("odo_wheel_pos: {}".format(odo_wheel_pos))
##
##        # Convert wheel distances to robot angle change of delta_theta radians around a point drive_radius meters to left of the robot
##        delta_theta = (left_wheel_distance - right_wheel_distance) / bot_wheel_distance # radians
##
##        if (delta_theta != 0):
##            drive_radius = 1/delta_theta * (right_wheel_distance + left_wheel_distance) / 2 # meter
##        else:
##            drive_radius = 9999
##
##
###        print("delta_theta: {}".format(delta_theta))
###        print("drive_radius: {}".format(drive_radius))
##
##        # Calculate to resulting position change in the robot coordinate system
##        if (drive_radius < 100):
##            dx_bot = drive_radius * (1 - cos(delta_theta))
##            dy_bot = drive_radius * sin(delta_theta)
##        else:
##            dx_bot = 0
##            dy_bot = (right_wheel_distance + left_wheel_distance) / 2
##
##        # Transform position change to world coordinate system
##        angle = (2*PI)- odo_orientation
##			
##        dx_world = cos(angle) * dx_bot - sin(angle) * dy_bot
##        dy_world = sin(angle) * dx_bot + cos(angle) * dy_bot
##        
##        odo_pos = [odo_pos[0] + dx_world, odo_pos[1] + dy_world]
##        odo_orientation = (odo_orientation + delta_theta) % (2*PI)
##        if (odo_orientation < 0):
##            odo_orientation += 2*PI;
##        
##        
##
##        
##        # Estimate velocity and position based on IMU-data
##        dv_x, dv_y, dv_z = dt * Acc[0], dt * Acc[1], dt * (Acc[2] - 9.81)
##        acc_vel = [acc_vel[0] + dv_x, acc_vel[1] + dv_y, acc_vel[2] + dv_z]
##
##        dx, dy, dz = acc_vel[0] * dt, acc_vel[1] * dt, acc_vel[2] * dt
##        acc_pos = [acc_pos[0] + dx, acc_pos[1] + dy, acc_pos[2] + dz]
##
###        print("Calculated: dT: {:04.2f}s, \nAcc-Velocity: {}, Acc-Position: {}".format(dt, [round(val, 2) for val in acc_vel], [round(val, 2) for val in acc_pos]))
###        print("Odo-Velocity: {}, Odo-Position: {}, Odo-Orientation: {}".format(odo_vel, odo_pos, odo_orientation))
##
##        last_time = time_now
except KeyboardInterrupt:
    verbindung.close()
    logfile.close()

print("Verbindung geschlossen")
