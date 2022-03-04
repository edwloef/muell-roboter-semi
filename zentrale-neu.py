import traceback
import sys
import RPi.GPIO as GPIO
import tflite_runtime.interpreter as tflite
import numpy as np
from picamera import PiCamera
from time import sleep
import cv2
import threading
from moveneu import Move
from sensor import Sensor



model_path = 'model.tflite'
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
        else:
            return None

def main():
    # init GPIO pins
    Move.init()
    Sensor.init()

    # test Sensor
    print(threading.Thread(target=Sensor.distance,args=()).start())
    
    # init camera object
    camera.start_preview()
    camera.resolution = (3280, 2464)
    update_image()

    # Load the TFLite model
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    try:
        while True:
            # Run inference
            vec_tuple = run_odt_and_draw_results(
                IMAGE_PATH,
                interpreter,
                threshold=DETECTION_THRESHOLD
            )

            update_image()

            if vec_tuple:
                if vec_tuple[0] < -328:
                    threading.Thread(target=Move.spin_left, args=()).start()
                elif vec_tuple[0] > 328:
                    threading.Thread(target=Move.spin_right, args=()).start()
                else:
                    threading.Thread(target=Move.drive_forwards, args=()).start()
                print("Object found.")
            else:
                threading.Thread(target=Move.spin_left, args=()).start()
                
    except KeyboardInterrupt:
        GPIO.cleanup()
        


if __name__ == "__main__":
    main()
