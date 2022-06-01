import RPi.GPIO as GPIO
import tflite_runtime.interpreter as tflite
import numpy as np
from picamera import PiCamera
import cv2
from moveneu import Move
from sensor import Sensor
import math
import random as r



model_path = 'paper_mit_augment.tflite'
DETECTION_THRESHOLD = 0.5
# Load the labels into a list
classes = ['Plastic', 'Paper', 'Glass']
label_map = ['Plastic', 'Paper', 'Glass']

camera = PiCamera()

IMAGE_PATH = 'img.jpg'

def update_image():
    camera.capture(IMAGE_PATH)






#==================================================================


def preprocess_image(image):
  """Preprocess the input image to feed to the TFLite model"""
  original_image = image
  resized_img = cv2.resize(image, (448,448))

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


def to_local_vector(x_min, x_max, y_min, y_max, original_image_shape): #das returnt im Moment noch nen global vec

  # das Koordinatensystem ist wieder mit y nach unten wachsend
  
  breite = original_image_shape[1] 
  hoehe = original_image_shape[0]


  vec_x = (((x_max-x_min)/2) + x_min) - (breite/2) # -(breite/2) shiftet das Ganze relativ zur Bildmitte --> globaler zu lokaler Vektor

  vec_y = hoehe - y_max #untere Boxkante  #da die y-Achse andersrum ist, hoehe - y_max

  vec_length_squared = vec_x**2 + vec_y**2 #quadriert eignet sich schon für den Längenvergleich, Wurzel muss nicht gezogen werden

  return (vec_x, vec_y, vec_length_squared)

def run_odt_and_draw_results(image_path, interpreter, threshold=0.5):
  """Run object detection on the input image and draw the detection results"""
  # Load the input shape required by the model
  _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']
  #print(input_height, input_width)
  # Load the input image and preprocess it
  preprocessed_image, original_image = preprocess_image(
      cv2.imread(image_path)
    )

  # Run object detection on the input image
  results = detect_objects(interpreter, preprocessed_image, threshold=threshold)

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

    vecs = [to_local_vector(x_min=xmin, x_max=xmax, y_min=ymin, y_max=ymax, original_image_shape=original_image_np.shape)]

   # get the shortest vec

    if(len(vecs) > 0):
        shortest_idx = 0
        shortest_length = vecs[0][2]

        for i in range(1, len(vecs)):
            if(vecs[i][2] < shortest_length):
                shortest_length = vecs[i][2]
                shortest_idx = i

        return (vecs[shortest_idx][0],vecs[shortest_idx][1])

    return None

def main():
    # init GPIO pins
    Move.init()
    Sensor.init()

    # test Sensor
    
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
            
            if at_object: # falls der Roboter schon Müll aufgesammelt hat
                while False: #! while Fahne not found
                    print(f"Sensor distance: {Sensor.distance()}")
                    if s.distance() > 10:
                        Move.drive_forwards(2)
                        Move.spin_left()
                    else:
                        Move.spin_left_angle(r.randint(90, 135))
                        #// Move.drive_forwards(5)
                # TODO zur Fahne fahren
                at_object = False
            elif vec_tuple: # falls Müll detektiert wurde
                print(f"Object detected: {vec_tuple}")
                angle = -0.3467 + 0.67 * (math.atan(abs(1640-vec_tuple[0])/(3555-vec_tuple[1])))
                angle = angle / (6.28) * 360
                print(f"Angle: {angle}")
                if angle < 0:
                    Move.spin_right_angle(-angle)
                else:
                    Move.spin_left_angle(angle)
                s = math.exp(25.81-2.8948*math.log(2464-vec_tuple[1]))
                print(f"Distance: {s}")
                s = min(100, s)
                Move.drive_forwards(s/Move.NORMAL_SPEED)
                at_object = True
            else:
                print("No object detected.")
                if Sensor.distance() > 10:
                    Move.drive_forwards(2)
                else:
                    Move.spin_left_angle(r.randint(90, 135))
            update_image()
    except KeyboardInterrupt:
        GPIO.cleanup()
        


if __name__ == "__main__":
    main()
