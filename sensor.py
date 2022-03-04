import RPi.GPIO as GPIO
from time import sleep


class Sensor():

    GPIO_TRIGGER = 12
    GPIO_ECHO = 23

    @staticmethod
    def distance():    
        # setze Trigger auf HIGH
        GPIO.output(GPIO_TRIGGER, True)
        # setze Trigger nach 0.01ms aus LOW
        sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        TriggerZeit=t.time()
        StartZeit = t.time()
        StopZeit = t.time()

        # speichere Startzeit
        while GPIO.input(GPIO_ECHO) == 0:
            StartZeit = t.time()
            if StartZeit-TriggerZeit>1:
                break
 
        # speichere Ankunftszeit
        while GPIO.input(GPIO_ECHO) == 1:
            StopZeit = t.time()
 
        # Zeit Differenz zwischen Start und Ankunft
        TimeElapsed = StopZeit - StartZeit
        # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
        # und durch 2 teilen, da hin und zurueck
        distance = (TimeElapsed * 34300) / 2

        if distance>450:
            return "Kein Hindernis in einer Entfernung von 400"
 
        return distanz
    
    @staticmethod
    def init(): 
        GPIO.setmode(GPIO.BCM)
 
        #Richtung der GPIO-Pins festlegen (IN / OUT)
        GPIO.setup(Sensor.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(Sensor.GPIO_ECHO, GPIO.IN)

        GPIO.output(Sensor.GPIO_TRIGGER,False)
