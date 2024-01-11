import serial
import time
import math

class Bot():

    connection = None
    sensors = None
    
    wheel_circ_corr_factor = 1
    invert_wheels = False
    
    def __init__(self):
        self.get_connection()
        self.reset_arduino()
        self.wait_for_arduino()
    
    def __del__(self):
        self.get_connection().close()
        
        
    def setInvertWheels(self, invertWheels):
        self.invert_wheels = invertWheels
        
    def setWheelCircumferenceCorrectionFactor(self, circumference_factor):
        self.wheel_circ_corr_factor = circumference_factor

    def get_connection(self):
        if self.connection == None:
            self.connection = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200, timeout=1, write_timeout=1)
        return self.connection

    def get_sensors(self):

        success = False
        while not success:
            self.get_connection().write("REQ")
            input = self.get_connection().readline()
            try:
                antwort = eval(input)
                
                if (not self.invert_wheels):
                    wheels_distance = {'left': long(antwort["Left_wheel_dist"])*self.wheel_circ_corr_factor/1000, 'right': long(antwort["Right_wheel_dist"])*self.wheel_circ_corr_factor/1000}
                    orientation = {'roll': antwort["IMU_roll"], 'pitch': antwort["IMU_pitch"], 'yaw': antwort["IMU_yaw"]+math.pi/2}
                else:
                    wheels_distance = {'left': -1*long(antwort["Right_wheel_dist"])*self.wheel_circ_corr_factor/1000, 'right': -1*long(antwort["Left_wheel_dist"])*self.wheel_circ_corr_factor/1000}
                    orientation = {'roll': antwort["IMU_roll"], 'pitch': antwort["IMU_pitch"], 'yaw': antwort["IMU_yaw"]}
                    
                
                acceleration = {'x': antwort["IMU_Acc_x"], 'y': antwort["IMU_Acc_y"], 'z': antwort["IMU_Acc_z"]}
                
                sensors = {'wheels_distance' : wheels_distance, 'orientation': orientation, 'acceleration': acceleration}
                success = True;
            except Exception as e:
                print(e)
                print("REQ didn't return a valid python dict")
                time.sleep(1)
        return sensors
        
    def flush_input(self):
        self.get_connection().flushInput()
        
        
    def set_wheel_speed(self, left_speed, right_speed):
    
        if (self.invert_wheels):
            left_speed, right_speed = -right_speed, -left_speed
            
        #left_speed *= self.wheel_circ_corr_factor
        #right_speed *= self.wheel_circ_corr_factor
    
    
        outstring = "CMDIN,"
        outstring += "{:05}".format(int(left_speed*1000))
        outstring += ","
        outstring += "{:05}".format(int(right_speed*1000))
        outstring += ",END\n"

        self.get_connection().write(outstring)
        
    def activate_joystick_mode(self):
        outstring = "JOYSTICK\n"
        self.get_connection().write(outstring)
        
    def reset_arduino(self):
        # Reset Arduino
        self.get_connection().setDTR(False)
        time.sleep(1)
        self.get_connection().flushInput()
        self.get_connection().setDTR(True)

    def wait_for_arduino(self):
        #Wait for Arduino to be ready
        start_time = time.time()
        while (time.time() - start_time) < 10: # Wait for Magnetometer/Gyro Fusion to fix
            self.get_connection().readline()