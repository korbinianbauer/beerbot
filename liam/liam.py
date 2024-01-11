from arduino_utils import Bot
from dead_reckoning import OdoDeadReckoner
from pure_pursuit import PurePursuitController
import time
import math
import random


bot = Bot()
bot.setInvertWheels(True)
bot.setWheelCircumferenceCorrectionFactor(0.971)

pathController = PurePursuitController(0.5948, 0.2, 0.1, 0.5)
deadReckoner = OdoDeadReckoner(0.5948, 0)


# RECORD
bot.activate_joystick_mode()
initial_yaw = bot.get_sensors()["orientation"]["yaw"]
#deadReckoner.setInitialYaw(initial_yaw)
deadReckoner.setInitialYaw(0) # East, for test path
start_time = time.time()
while(time.time() - start_time < 2):
    #bot.set_wheel_speed(1.0,1.0)
    sensors = bot.get_sensors()
    
    deadReckoner.evaluate(sensors["wheels_distance"]["left"], sensors["wheels_distance"]["right"])

    time.sleep(0.1)
    
bot.set_wheel_speed(0,0)
    
# RETRIEVE
path_raw = deadReckoner.getPosHistory()[:] # independent copy
path = []




for waypoint_raw in path_raw:
    x = waypoint_raw[0]
    y = waypoint_raw[1]
    vel = waypoint_raw[3]
    
    waypoint = {'x': x, 'y': y, 'vel': vel}
    path.append(waypoint)

path.reverse()  
del path[0]  
    
# TESTPATH

path = []
for i in range (100):
    x = i/100.0
    y = 0
    
    vel = 1
    waypoint = {'x': x, 'y': y, 'vel': vel}
    path.append(waypoint)
    
'''
for i in range (100):
    x = 1.0
    y = i/100.0
    
    vel = 1
    waypoint = {'x': x, 'y': y, 'vel': vel}
    path.append(waypoint)
'''

# debugging
#for waypoint in path:
#    print(waypoint)

# REPLAY
pathController.setPath(path)
target_pos = pathController.getPathEnd()
start_time = time.time()
while(pathController.getDistanceToTarget() > 0.05):
    sensors = bot.get_sensors()
    current_pos = deadReckoner.evaluate(sensors["wheels_distance"]["left"], sensors["wheels_distance"]["right"])
    wheels_velocities = pathController.evaluate({'x': current_pos[0], 'y': current_pos[1]}, current_pos[2])
    bot.set_wheel_speed(*wheels_velocities)
    #print("target_pos: {}, Distance to target: {}".format(target_pos, pathController.getDistanceToTarget()))
    time.sleep(0.01)

   
bot.set_wheel_speed(0,0)
