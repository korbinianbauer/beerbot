import serial
import time

def log_bot():
    global verbindung
    verbindung.write("REQ")
    antwort = verbindung.readline()
    return antwort
    
    
def reset_arduino():
    # Reset Arduino
    verbindung.setDTR(False)
    time.sleep(1)
    verbindung.flushInput()
    verbindung.setDTR(True)

def wait_for_arduino():
    #Wait for Arduino to be ready
    start_time = time.time()
    while (time.time() - start_time) < 10: # Wait for Magnetometer/Gyro Fusion to fix
        verbindung.readline()
        
        
def set_wheel_speed(left_speed, right_speed):
    outstring = "CMDIN,"
    outstring += "{:05}".format(int(left_speed*1000))
    outstring += ","
    outstring += "{:05}".format(int(right_speed*1000))
    outstring += ",END\n"
    
    print(outstring)

    global verbindung
    verbindung.write(outstring)


verbindung = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200, timeout=1, write_timeout=1)


reset_arduino()
wait_for_arduino()
set_wheel_speed(0.5, 0.5)
print(log_bot())
time.sleep(1)
set_wheel_speed(0, 0)
print(log_bot())
