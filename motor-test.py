import RPi.GPIO as GPIO
import time
import sys

motor_1_channel = (5, 13, 6, 19)
motor_2_channel = (26, 16, 20, 21)
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_1_channel, GPIO.OUT)
GPIO.setup(motor_2_channel, GPIO.OUT)

for pin in motor_1_channel + motor_2_channel:
    GPIO.output(pin, 0)

halfstep_seq = [[1,0,0,0],[1,0,0,1],[0,0,0,1],[0,0,1,1],[0,0,1,0],[0,1,1,0],[0,1,0,0],[1,1,0,0]]

while True:
    try:
        for halfstep in halfstep_seq:
            GPIO.output(motor_1_channel, halfstep)
            GPIO.output(motor_2_channel, halfstep)
            time.sleep(0.0006)
    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit(0)
