def crosstalk():    
    GPIO.output(GPIO_TRIGGER_RIGHT, True)
    GPIO.output(GPIO_TRIGGER_LEFT, True) # setze Trigger auf HIGH
    sleep(0.00001) # setze Trigger nach 0.01ms auf LOW
    #! wieso? ^^
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    TriggerZeit = time()
    StartZeit1 = time()
    StopZeit1 = time()
    StartZeit2 = time()
    StopZeit2 = time()
    
    x = False
    x2=False
    y=False
    y2 = False

    # speichere Startzeit
    while not x2 or not y2 :
        if GPIO.input(GPIO_ECHO_RIGHT) == 0 and not x:
            StartZeit1 = time()
        elif GPIO.input(GPIO_ECHO_RIGHT) == 0 and x:
            x2=True
        elif GPIO.input(GPIO_ECHO_RIGHT)==1:
            StopZeit1=time()
            x=True
        if GPIO.input(GPIO_ECHO_LEFT) == 0 and not y:
            StartZeit2 = time()
        elif GPIO.input(GPIO_ECHO_LEFT) == 0 and y:
            y2=True
        elif GPIO.input(GPIO_ECHO_LEFT) == 1:
            StopZeit2=time()
            y=True
        if StartZeit1-TriggerZeit>1 or StartZeit2-TriggerZeit>1:
            break

    # speichere Ankunftszeit
    #while GPIO.input( GPIO_ECHO_RIGHT) == 1:
       # StopZeit = time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed1 = StopZeit1 - StartZeit1
    print(TimeElapsed1)
    # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
    # und durch 2 teilen, da hin und zurueck
    distance1 = (TimeElapsed1 * 34300) / 2
    TimeElapsed2 = StopZeit2 - StartZeit2
    print(TimeElapsed2)
        # mit der Schallgeschwindigkeit (34300 cm/s) multiplizieren
        # und durch 2 teilen, da hin und zurueck
    distance2 = (TimeElapsed2 * 34300) / 2
    

    #if distance>450:
        #return "Kein Hindernis in einer Entfernung von 400"
    #distance = r.randint(10,12)
    return min(distance1,distance2)
