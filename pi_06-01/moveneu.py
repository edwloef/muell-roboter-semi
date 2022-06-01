import RPi.GPIO as GPIO
import numpy
import time as t


class Move():
    
    HALFSTEPS = ((1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1))

    # motor pins
    LMOTOR = (13, 19, 5, 6)
    RMOTOR = (16, 26, 20, 21)

    #halber Abstand der Räder
    RADIUS = 5.527
    #speed mit sleeptime = 0.001
    NORMAL_SPEED = 4.424
    #für Ausführung eines steps benötigte zeit
    STEP_TIME = 0.000839

    @staticmethod
    def move(left, right, time, speed) -> None:
        if not time:
            time = 5
        if not speed:
            speed = Move.NORMAL_SPEED
        sleeptime = Move.NORMAL_SPEED / (speed * 1000)
        if sleeptime<0.0009:
            sleeptime=0.0009
        steps = time / (8*sleeptime+Move.STEP_TIME)
        
        """        
        if sleeptime<0.0007:
            steps = (time-0.724) / (8*sleeptime+Move.STEP_TIME)  #mit 50 mal 0.0009 und 50 mal 0.0007 als anschub
            for i in range(50):
                for l,r in zip(left,right):
                    GPIO.output(Move.LMOTOR, l)
                    GPIO.output(Move.RMOTOR, r)
                    t.sleep(0.0009)
            for i in range(50):
                for l,r in zip(left,right):
                    GPIO.output(Move.LMOTOR, l)
                    GPIO.output(Move.RMOTOR, r)
                    t.sleep(0.0007)
        elif sleeptime<0.0009:
            steps = (time-0.402) / (8*sleeptime+Move.STEP_TIME)  #mit 50 mal 0.0009 als anschub 
            for i in range(50):
                for l,r in zip(left,right):
                    GPIO.output(Move.LMOTOR, l)
                    GPIO.output(Move.RMOTOR, r)
                    t.sleep(0.0009)
        """

        for i in range(round(steps)):
            for l,r in  zip(left,right):
                GPIO.output(Move.LMOTOR, l)
                GPIO.output(Move.RMOTOR, r)
                t.sleep(sleeptime)
    
    #time in s; speed in cm/s
    @staticmethod
    def drive_backwards(time = None, speed = None):      
        Move.move(Move.HALFSTEPS,Move.HALFSTEPS[::-1], time, speed)

    #time in s; speed in cm/s
    @staticmethod
    def drive_forwards(time=None,speed=None):
        Move.move(Move.HALFSTEPS[::-1],Move.HALFSTEPS,time, speed)

    #time in s; speed in Hz
    @staticmethod
    def spin_left(time = None, speed = None):
        if not speed:
            speed = Move.NORMAL_SPEED/Move.RADIUS
        speed = speed * Move.RADIUS
        Move.move(Move.HALFSTEPS[::-1], Move.HALFSTEPS[::-1], time, speed)

    #time in s; speed in Hz
    @staticmethod
    def spin_right(time = None, speed = None):
        if not speed:
            speed = Move.NORMAL_SPEED/Move.RADIUS
        speed = speed * Move.RADIUS 
        Move.move(Move.HALFSTEPS, Move.HALFSTEPS, time, speed)

    #angle in °
    @staticmethod
    def spin_left_angle(angle):
        speed = Move.NORMAL_SPEED
        time = 2 * numpy.pi * Move.RADIUS * angle / (speed*360)
        Move.move(Move.HALFSTEPS[::-1], Move.HALFSTEPS[::-1], time, speed)

    #angle in °
    @staticmethod
    def spin_right_angle(angle):
        speed = Move.NORMAL_SPEED
        time = 2 * numpy.pi * Move.RADIUS * angle / (speed*360)
        Move.move(Move.HALFSTEPS, Move.HALFSTEPS, time, speed)
    
    @staticmethod
    def init(): 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Move.RMOTOR + Move.LMOTOR, GPIO.OUT)
        for pin in Move.RMOTOR + Move.LMOTOR:
            GPIO.output(pin, 0)
