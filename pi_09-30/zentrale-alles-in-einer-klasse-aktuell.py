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


GPIO_TRIGGER_LEFT = 12
GPIO_ECHO_LEFT = 23
GPIO_TRIGGER_RIGHT = 12
GPIO_ECHO_RIGHT = 23

def distance(gpio_trigger, gpio_echo):    
    GPIO.output(gpio_trigger, True) # setze Trigger auf HIGH
    sleep(0.00001) # setze Trigger nach 0.01ms auf LOW
    #! wieso? ^^
    GPIO.output(gpio_trigger, False)
    TriggerZeit = time()
    StartZeit = time()
    StopZeit = time()

    # speichere Startzeit
    while GPIO.input(gpio_echo) == 0:
        StartZeit = time()
        if StartZeit-TriggerZeit>1:
            break

    # speichere Ankunftszeit
    while GPIO.input(gpio_echo) == 1:
        StopZeit = time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
    # und durch 2 teilen, da hin und zurueck
    distance = (TimeElapsed * 34300) / 2

    #if distance>450:
        #return "Kein Hindernis in einer Entfernung von 400"
    distance = r.randint(10,12)
    return distance

def distanceLeft():
    return distance(GPIO_TRIGGER_LEFT, GPIO_ECHO_LEFT)

def distanceRight():
    return distance(GPIO_TRIGGER_RIGHT, GPIO_ECHO_RIGHT)
    

    

#==================================================================

    
HALFSTEPS = ((1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1))
HALFSTEPS_HALFSPEED = ((1, 0, 0, 0), (1, 0, 0, 0),  (1, 1, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0), (0, 1, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 1, 1), (0, 0, 0, 1), (0, 0, 0, 1), (1, 0, 0, 1), (1, 0, 0, 1))

# motor pins
LMOTOR = (13, 19, 5, 6)
RMOTOR = (16, 26, 20, 21)

RADIUS = 5.527 # halber Abstand der Räder, bei Kurven das dreifache
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

def curve_left(angle):
    speed = NORMAL_SPEED
    time = 4* math.pi*RADIUS*angle/(speed*360)
    move(HALFSTEPS[::-1] + HALFSTEPS[::-1], HALFSTEPS_HALFSPEED, time, speed)

def curve_right(angle):
    speed = NORMAL_SPEED
    time = 4* math.pi*RADIUS*angle/(speed*360)
    move(HALFSTEPS_HALFSPEED[::-1], HALFSTEPS + HALFSTEPS, time, speed)


#==================================================================

def detect_pink_flag(image_path):  
    img = cv2.imread(image_path)
    # convert to hsv colorspace 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    a = 39
    b = 75    

    lower_bound = np.array([129-b,112-b,204-b],np.uint8)    
    upper_bound = np.array([149+a,132+a,224+a],np.uint8)  
    # lower_bound = np.array([0,0,0],np.uint8)    
    #upper_bound = np.array([178,245,254],np.uint8) 
    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    #define kernel size  
    kernel = np.ones((7,7),np.uint8)
    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Segment only the detected region
    segmented_img = cv2.bitwise_and(img, img, mask=mask)

    height, width = img.shape[:2]


    x=[]
    y=[]

    d = 50
    for i in range(0,height,1):
        for j in range(0,width,d):
            if segmented_img[i][j][0] > 0:
                x.append(j)
                y.append(i)
                j-= 25 #wenn pink gefunden nur noch einerschritte, sonst immer zweierschritte
                  
    if(len(x)<1):    
        return False
    x = np.median(x)
    y = np.median(y)
    #cv2.imshow("Output", segmented_img)

     #zum lokalen vektor (von unten Mitte ausgehend)....geht das dann mit der konverter funktion noch? (es geht auf jeden fall, wenn die breite und höhe kleiner als 448 sind)
    
    x = x = x-width/2
    y = height-y

   # cv2.waitKey(0)
   # cv2.destroyAllWindows()

    return (x,y)
#=======================================================================

def sidestep(s):
    if (min(distanceLeft(), distanceRight()) < s):
        if (distanceLeft() >= distanceRight()):
            curve_left(45)
            if (min(distanceLeft(), distanceRight()) > (3/4*s)/math.sqrt(2)):#siehe if vec_tuple
                drive_forwards((3/4*s)/math.sqrt(2))
                curve_right(90)
                #hier eigentlich auch nochmal schaun
                if (min(distanceLeft(), distanceRight()) > (3/4*s)/math.sqrt(2)):
                    drive_forwards((3/4*s)/math.sqrt(2))
                    curve_left(45)
                else:
                    return False
            else:
                return False
        else:
            curve_right(45)
            if (min(distanceLeft(), distanceRight()) > (3/4*s)/math.sqrt(2)):
                drive_forwards((3/4*s)/math.sqrt(2))
                curve_left(90)
                if (min(distanceLeft(), distanceRight()) > (3/4*s)/math.sqrt(2)):
                    drive_forwards((3/4*s)/math.sqrt(2))
                    curve_right(45)
                else:
                    return False
            else:
                return False
        drive_forwards(1/4*s)
    else:
        drive_forwards(s/NORMAL_SPEED)
    return True

#=========================================================================

def main():
    GPIO.setmode(GPIO.BCM)
    
    # init left sensor 
    GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
    GPIO.output(GPIO_TRIGGER_LEFT,False)

    # init right sensor 
    GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
    GPIO.output(GPIO_TRIGGER_RIGHT,False)
    
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
        fahne = False
        while True:
            # Run inference
            vec_tuple = run_odt_and_draw_results(
                IMAGE_PATH,
                interpreter,
                threshold=DETECTION_THRESHOLD
            )

            while at_object:  # falls der Roboter schon Müll aufgesammelt hat
                while fahne == False: #! while Fahne not found
                    print(f"Sensor distance: {distance()}")
                    if min(distanceLeft(), distanceRight()) < 10:
                        ratio = 0
                        left = False
                        if distanceLeft() <= distanceRight():#ermittelt Verhältnis und ob sich der Roboter nach links oder rechts drehen muss
                            ratio = distanceRight()/distanceLeft()
                            
                        else:
                            ratio = distanceLeft()/distanceRight()
                            left = True

                        if ratio < 1.3:# bei einem Verhältnis unter 1,3 dreht sich der Roboter erst, ermittelt erneut das Verhältnis und kann so entscheiden, ob er an eine Wand oder eine Ecke gefahren ist
                            if left:
                                curve_left(45)
                                ratio = distanceLeft()/distanceRight()
                                if ratio < 1.5:
                                    curve_left(135)
                                else:
                                    curve_left(55)

                            else:
                                curve_right(45)
                                ratio = distanceRight()/distanceLeft()
                                if ratio < 1.5:
                                    curve_right(135)
                                elif ratio < 2:
                                    curve_right(55)
                        else:
                            curve_left(r.randint(90, 135))#Roboter dreht sich zufällig zwischen 90 und 135°
                    else:
                        drive_forwards(2)#ist kein Hindernis in der Nähe, so kann der Roboter geradeaus fahren
                    update_image()
                    vec_fahne = detect_pink_flag(IMAGE_PATH) 
                    if vec_fahne:# janniks zeug
                        fahne = True
                        print("Fahne gefunden")
                    else:
                        print("nicht gefunden")
                angle = 0.8 * (math.atan(vec_fahne[0])/(3712-vec_fahne[1]))
                angle = angle / (6.28) * 360
                print(f"winkel{angle}")
                if angle <-3:
                    spin_left_angle(-angle)
                elif angle >3:
                    spin_right_angle(angle)
                else:
                    s = math.exp(26.63-3.0089*math.log(2464-vec_fahne[1]))
                    print(f"Strecke:{s}")
                    if s>100:
                        sidestep(100)

                    else:
                        if sidestep(s-10):
                            drive_backwards(6)
                            spin_left_angle(180)
                            at_object = False
                            fahne = False
                if fahne:
                    update_image()
                    vec_fahne = detect_pink_flag(IMAGE_PATH) 
                    if vec_fahne:# janniks zeug
                        print("Fahne gefunden")
                    else:
                        fahne = False
                        print("nicht gefunden")
            if vec_tuple:  # falls Müll detektiert wurde
                print(f"Object detected: {vec_tuple}")
                # vec_tuple[0] -> x-Koordinate von mitte
                # vec_tuple[1] -> y-Koordinate von oben
                angle = 0.8 * math.atan(vec_tuple[0]/(3712-vec_tuple[1]))
                angle = angle / (6.28) * 180
                print(f"Angle: {angle}")
                if angle >1:
                    spin_right_angle(angle)
                elif angle <-1:
                    spin_left_angle(-angle)
                else:
                    s = math.exp(26.63-3.0089*math.log(2464-vec_tuple[1]))
                    print(f"Distance: {s}")
                    if s>100:
                        sidestep(100)
                    else:
                        if sidestep(s):
                            at_object = True                        

            else:
                print("No object detected.")
                if min(distanceLeft(), distanceRight()) > 10:
                    if r.randint(3) == 0:#in 2 von 3 Fällen fährt er geradeaus, in einem Fall dreht er sich zwischen 50 und 100°
                        spin_left_angle(r.randint(50, 100))
                    else:
                        drive_forwards(2)
                    
                else:
                    spin_left_angle(r.randint(90, 135))
            update_image()
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
