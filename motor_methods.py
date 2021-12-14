import RPi.GPIO as GPIO
import time as t

halfstep_seq = [[1, 0, 0, 0], [1, 0, 0, 1], [0, 0, 0, 1], [
    0, 0, 1, 1], [0, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 0], [1, 1, 0, 0]]
halfstep_seq_reverse = [[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [
    0, 1, 1, 0], [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]]
empty_steps = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [
    0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
motor_r = (13, 19, 5, 6)
motor_l = (16, 26, 20, 21)

def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor_r, GPIO.OUT)
    GPIO.setup(motor_l, GPIO.OUT)
    for pin in motor_r + motor_l:
        GPIO.output(pin, 0)

def move(left: list, right: list, time: int = 64, speed: float = 0.001):
    for _ in range(time):
        for l, r in zip(left, right):
            GPIO.output(motor_r, l)
            GPIO.output(motor_l, r)
            t.sleep(speed)
    GPIO.cleanup()


def drive_forwards(time: int = None, speed: float = None):
    move(halfstep_seq_reverse, halfstep_seq, time, speed)


def drive_backwards(time: int = None, speed: float = None):
    move(halfstep_seq, halfstep_seq_reverse, time, speed)


def turn_left_forwards(time: int = None, speed: float = None):
    move(halfstep_seq_reverse, empty_steps, time, speed)


def turn_right_forwards(time: int = None, speed: float = None):
    move(empty_steps, halfstep_seq, time, speed)


def turn_left_backwards(time: int = None, speed: float = None):
    move(halfstep_seq, empty_steps, time, speed)


def turn_right_backwards(time: int = None, speed: float = None):
    move(empty_steps, halfstep_seq_reverse, time, speed)


def spin_left(time: int = None, speed: float = None):
    move(halfstep_seq, halfstep_seq, time, speed)


def spin_right(time: int = None, speed: float = None):
    move(halfstep_seq_reverse, halfstep_seq_reverse, time, speed)