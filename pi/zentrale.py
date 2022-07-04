import RPi.GPIO as GPIO
import tflite_runtime.interpreter as tflite
import numpy as np
from picamera import PiCamera
import cv2
from moveneu import Move
from sensor import Sensor
from time import sleep, time
import math
import random as r

model_path = 'paper_mit_augment.tflite'
DETECTION_THRESHOLD = 0.5
# Load the labels into a list
classes = ['Plastic', 'Paper', 'Glass']
label_map = ['Plastic', 'Paper', 'Glass']

# ==================================================================


camera = PiCamera()

IMAGE_PATH = 'img.jpg'

def update_image():
    camera.capture(IMAGE_PATH)


#==================================================================


def preprocess_image(image):
    """Preprocess the input image to feed to the TFLite model"""
    original_image = image
    resized_img = cv2.resize(image, (448, 448))

    resized_img = np.expand_dims(np.array(resized_img), 0)
   # print(resized_img.shape)

    return resized_img, original_image


def detect_objects(interpreter, image, threshold):
    """Returns a list of detection results, each a dictionary of object info."""

    signature_fn = interpreter.get_signature_runner()

    # Feed the input image to the model
    output = signature_fn(images=image)

    # Get all outputs from the model
    count = int(np.squeeze(output['output_0']))
    scores = np.squeeze(output['output_1'])
    classes = np.squeeze(output['output_2'])
    boxes = np.squeeze(output['output_3'])

    results = []
    for i in range(count):
        if scores[i] >= threshold:
            result = {
                'bounding_box': boxes[i],
                'class_id': classes[i],
                'score': scores[i]
            }
            results.append(result)
    return results


# das returnt im Moment noch nen global vec
def to_local_vector(x_min, x_max, y_min, y_max, original_image_shape):

    # das Koordinatensystem ist wieder mit y nach unten wachsend

    breite = original_image_shape[1]
    hoehe = original_image_shape[0]

    # -(breite/2) shiftet das Ganze relativ zur Bildmitte --> globaler zu lokaler Vektor
    vec_x = (((x_max-x_min)/2) + x_min) - (breite/2)

    vec_y = hoehe - y_max  # untere Boxkante  #da die y-Achse andersrum ist, hoehe - y_max

    # quadriert eignet sich schon für den Längenvergleich, Wurzel muss nicht gezogen werden
    vec_length_squared = vec_x**2 + vec_y**2

    return (vec_x, vec_y, vec_length_squared)


def run_odt_and_draw_results(image_path, interpreter, threshold=0.5):
    """Run object detection on the input image and draw the detection results"""
    # Load the input shape required by the model
    _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']
    #print(input_height, input_width)
    # Load the input image and preprocess it
    preprocessed_image, original_image = preprocess_image(cv2.imread(image_path))

    # Run object detection on the input image
    results = detect_objects(
        interpreter, preprocessed_image, threshold=threshold)

    # Plot the detection results on the input image
    original_image_np = original_image.astype(np.uint8)
    for obj in results:
        # Convert the object bounding box from relative coordinates to absolute
        # coordinates based on the original image resolution
        ymin, xmin, ymax, xmax = obj['bounding_box']
        xmin = int(xmin * original_image_np.shape[1])
        xmax = int(xmax * original_image_np.shape[1])
        ymin = int(ymin * original_image_np.shape[0])
        ymax = int(ymax * original_image_np.shape[0])

        vecs = [to_local_vector(x_min=xmin, x_max=xmax, y_min=ymin,
                                y_max=ymax, original_image_shape=original_image_np.shape)]

       # get the shortest vec
        if(len(vecs) > 0):
            shortest_idx = 0
            shortest_length = vecs[0][2]

            for i in range(1, len(vecs)):
                if(vecs[i][2] < shortest_length):
                    shortest_length = vecs[i][2]
                    shortest_idx = i

            return (vecs[shortest_idx][0], vecs[shortest_idx][1])

        return None


#==================================================================


GPIO_TRIGGER = 12
GPIO_ECHO = 23

def distance():    
    GPIO.output(GPIO_TRIGGER, True) # setze Trigger auf HIGH
    sleep(0.00001) # setze Trigger nach 0.01ms auf LOW
    #! wieso? ^^
    GPIO.output(GPIO_TRIGGER, False)
    TriggerZeit = time()
    StartZeit = time()
    StopZeit = time()

    # speichere Startzeit
    while GPIO.input(GPIO_ECHO) == 0:
        StartZeit = time()
        if StartZeit-TriggerZeit>1:
            break

    # speichere Ankunftszeit
    while GPIO.input(GPIO_ECHO) == 1:
        StopZeit = time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
    # und durch 2 teilen, da hin und zurueck
    distance = (TimeElapsed * 34300) / 2

    if distance>450:
        return "Kein Hindernis in einer Entfernung von 400"

    return distance
    

#==================================================================

    
HALFSTEPS = ((1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1))

# motor pins
LMOTOR = (13, 19, 5, 6)
RMOTOR = (16, 26, 20, 21)

RADIUS = 5.527 # halber Abstand der Räder
NORMAL_SPEED = 4.424 # speed mit sleeptime = 0.001
STEP_TIME = 0.000839  # für Ausführung eines steps benötigte Zeit


def move(left, right, time, speed) -> None:
    if not time:
        time = 5
    if not speed:
        speed = NORMAL_SPEED
    sleeptime = NORMAL_SPEED / (speed * 1000)
    if sleeptime<0.0009:
        sleeptime=0.0009
    steps = time / (8*sleeptime+STEP_TIME)
    
    """        
    if sleeptime<0.0007:
        steps = (time-0.724) / (8*sleeptime+STEP_TIME)  #mit 50 mal 0.0009 und 50 mal 0.0007 als anschub
        for i in range(50):
            for l,r in zip(left,right):
                GPIO.output(LMOTOR, l)
                GPIO.output(RMOTOR, r)
                sleep(0.0009)
        for i in range(50):
            for l,r in zip(left,right):
                GPIO.output(LMOTOR, l)
                GPIO.output(RMOTOR, r)
                sleep(0.0007)
    elif sleeptime<0.0009:
        steps = (time-0.402) / (8*sleeptime+STEP_TIME)  #mit 50 mal 0.0009 als anschub 
        for i in range(50):
            for l,r in zip(left,right):
                GPIO.output(LMOTOR, l)
                GPIO.output(RMOTOR, r)
                sleep(0.0009)
    """

    for i in range(round(steps)):
        for l,r in  zip(left,right):
            GPIO.output(LMOTOR, l)
            GPIO.output(RMOTOR, r)
            sleep(sleeptime)


# time in s; speed in cm/s
def drive_backwards(time = None, speed = None):      
    move(HALFSTEPS,HALFSTEPS[::-1], time, speed)


# time in s; speed in cm/s
def drive_forwards(time=None,speed=None):
    move(HALFSTEPS[::-1],HALFSTEPS,time, speed)


# time in s; speed in Hz
def spin_left(time = None, speed = None):
    if not speed:
        speed = NORMAL_SPEED/RADIUS
    speed = speed * RADIUS
    move(HALFSTEPS[::-1], HALFSTEPS[::-1], time, speed)


# time in s; speed in Hz
def spin_right(time = None, speed = None):
    if not speed:
        speed = NORMAL_SPEED/RADIUS
    speed = speed * RADIUS 
    move(HALFSTEPS, HALFSTEPS, time, speed)


# angle in °
def spin_left_angle(angle):
    speed = NORMAL_SPEED
    time = 2 * math.pi * RADIUS * angle / (speed*360)
    move(HALFSTEPS[::-1], HALFSTEPS[::-1], time, speed)


# angle in °
def spin_right_angle(angle):
    speed = NORMAL_SPEED
    time = 2 * math.pi * RADIUS * angle / (speed*360)
    move(HALFSTEPS, HALFSTEPS, time, speed)


#==================================================================


def main():
    GPIO.setmode(GPIO.BCM)
    
    # init sensor
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)
    GPIO.output(GPIO_TRIGGER,False)
    
    # init motors
    GPIO.setup(RMOTOR + LMOTOR, GPIO.OUT)
    for pin in RMOTOR + LMOTOR:
        GPIO.output(pin, 0)

    # init camera object
    camera.start_preview()
    camera.resolution = (3280, 2464)
    update_image()

    # Load the TFLite model
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    try:
        at_object = False
        while True:
            # Run inference
            vec_tuple = run_odt_and_draw_results(
                IMAGE_PATH,
                interpreter,
                threshold=DETECTION_THRESHOLD
            )

            if at_object:  # falls der Roboter schon Müll aufgesammelt hat
                while False:  # ! while Fahne not found
                    print(f"Sensor distance: {distance()}")
                    if s.distance() > 10:
                        drive_forwards(2)
                        spin_left()
                    else:
                        spin_left_angle(r.randint(90, 135))
                        #// drive_forwards(5)
                #! zur Fahne fahren
                at_object = False
            elif vec_tuple:  # falls Müll detektiert wurde
                print(f"Object detected: {vec_tuple}")
                # vec_tuple[0] -> x-Koordinate von mitte
                # vec_tuple[1] -> y-Koordinate von oben
                angle = 0.8 * math.atan(vec_tuple[0]/(3555-vec_tuple[1]))
                angle = angle / (6.28) * 360
                print(f"Angle: {angle}")
                if angle < -3:
                    spin_left_angle(-angle)
                elif angle > 3:
                    spin_right_angle(angle)
                else:
                    s = math.exp(25.81-2.8948*math.log(2464-vec_tuple[1]))
                    #print(f"Distance: {s}")
                    s = min(100, s) + 5
                    drive_forwards(s/NORMAL_SPEED)
                    at_object = True
            else:
                print("No object detected.")
                if distance() > 10:
                    drive_forwards(2)
                else:
                    spin_left_angle(r.randint(90, 135))
            update_image()
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
    