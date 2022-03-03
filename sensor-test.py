#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 11 16:28:58 2021

@author: mint-user
"""
#Bibliotheken einbinden
import RPi.GPIO as GPIO
import time as t
 
#GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#GPIO Pins zuweisen
GPIO_TRIGGER = 12
GPIO_ECHO = 23
 
#Richtung der GPIO-Pins festlegen (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

GPIO.output(GPIO_TRIGGER,False)
 
def distanz():
    # setze Trigger auf HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # setze Trigger nach 0.01ms aus LOW
    t.sleep(0.00001)
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
    distanz = (TimeElapsed * 34300) / 2

    if distanz>450:
        return "Kein Hindernis in einer Entfernung von 400"
 
    return distanz
 
if __name__ == '__main__':
    try:
        while True:
            abstand = distanz()
            print(f"Gemessene Entfernung = {abstand} cm")
            t.sleep(2.5)
 
        # Beim Abbruch durch STRG+C resetten
    except KeyboardInterrupt:
        print("Messung vom User gestoppt")
        GPIO.cleanup()
