import serial
import time

verbindung = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55736303831351F03132-if00', 115200)
time.sleep(5)

verbindung.write('text')
try:
    while True:
        antwort = verbindung.readline()
        print(antwort)
except KeyboardInterrupt:
    verbindung.close()