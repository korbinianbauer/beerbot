import serial
import time

verbindung = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200)
time.sleep(5)

verbindung.write('text')

pos = [0,0,0]
vel = [0,0,0]
last_time = 0
time_now = 0

try:
    while True:
        antwort = verbindung.readline()
        data = antwort.replace('\n', '').split(',')

        

        if (data[0] == "TIM_OUT" and data[2] == "END"):
            #print("Time data detected!")

            time_now = float(data[1])

            print(time_now)

            if last_time == 0:
                last_time = time_now

        if (0 == last_time or 0 == time_now): # No valid time signal detected yet, don't evaluate Acc
            continue

        if (data[0] == "ACC_OUT" and data[4] == "END"):
            #print("Acc data detected!")

            Acc = [float(val) for val in data[1:4]]
            #print("Acceleration values: ")
            #print(Acc)

            # Add dt*Acc to vel
            dt = time_now - last_time
            print("dt: ")
            print(dt)
            dv_x, dv_y, dv_z = dt * Acc[0], dt * Acc[1], dt * (Acc[2] - 9.81)

            vel = [vel[0] + dv_x, vel[1] + dv_y, vel[2] + dv_z]

            dx, dy, dz = vel[0] * dt, vel[1] * dt, vel[2] * dt

            pos = [pos[0] + dx, pos[1] + dy, pos[2] + dz]

            print("Acceleration: ")
            print(Acc)
            print("Velocity: ")
            print(vel)

            last_time = time_now
except KeyboardInterrupt:
    verbindung.close()
