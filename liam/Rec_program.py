import serial
import time
import sys
from programs import programs, program_descs

program_to_run = 0
if (len(sys.argv) > 1):
    try:
        program_to_run = int(sys.argv[1])
    except:
        program_to_run = sys.argv[1]
else:
    print("Available programs:")
    for i, prg_desc in enumerate(program_descs):
        print("{}: {}".format(i, prg_desc))
    print("\nUsage: python {} [program number]".format(sys.argv[0]))
    sys.exit()

instruction_interval = 0.1

logfile = None
last_instruction_sent = 0
verbindung = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200, timeout=1, write_timeout=1)

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

def set_logfile():
    filename = "/home/pi/beerbot/liam/log/"
    filename += time.ctime(time.time())
    
    filename += "_Program_"
    filename += str(program_to_run)
    filename = filename.replace(":", "-")
    filename = filename.replace(" ", "_")
    filename += ".txt"
    print("Saving log as {}".format(filename))
    global logfile
    logfile = open(filename,"w")


def log_bot():
    global verbindung
    verbindung.write("REQ")
    antwort = verbindung.readline()
    logfile.write(antwort)

def log_prog_desc():
    global verbindung
    logfile.write(time.ctime(time.time()) + "\n")
    logfile.write(program_descs[program_to_run] + "\n")
    
def log_prog_desc_joystick():
    global verbindung
    logfile.write(time.ctime(time.time()) + "\n")
    logfile.write("Joystick Mode (120s)" + "\n")


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
                del program[0]
                
                logfile.close()
            return

def activate_joystick_mode():
    outstring = "JOYSTICK\n"
    global verbindung
    verbindung.write(outstring)
    
def drive_by_joystick(time_t):
    global last_instruction_sent
    global instruction_interval
    if (last_instruction_sent + instruction_interval) > time_t:
        return
    
    last_instruction_sent = time_t
    log_bot()
    
            
if ("joystick" == program_to_run):
    # Joystick mode
    print("Running joystick record mode for 120s")
    print("\nResetting Arduino and waiting for sensor fusion to settle in...")
    reset_arduino()
    wait_for_arduino()
    set_logfile()
    log_prog_desc_joystick()
    print("Running...")
    activate_joystick_mode()
    start_time = time.time()
    try:
        while time.time() < start_time + 120:
            time_now = time.time() - start_time
            drive_by_joystick(time_now)
            
        logfile.close()
        print("Finished program")
        sys.exit()

    except KeyboardInterrupt:
        verbindung.close()
        logfile.close()
        sys.exit()

print("Running {}".format(program_descs[program_to_run]))
print(str(programs[program_to_run]).replace("],", "],\n"))
print("\nResetting Arduino and waiting for sensor fusion to settle in...")
reset_arduino()
wait_for_arduino()
set_logfile()
log_prog_desc()
print("Running...")
start_time = time.time()
try:
    while len(programs[program_to_run]):
        time_now = time.time() - start_time
        drive_by_program(programs[program_to_run], time_now)
    print("Finished program")

except KeyboardInterrupt:
    verbindung.close()
    logfile.close()
