import numpy as np
#import matplotlib.pyplot as plt
#from matplotlib.transforms import Bbox
#from matplotlib.path import Path

np.seterr(divide='ignore') #division durch 0 abfangen, weil die schon anderweitig "beachtet" wird

def intersection(ray_start_x, ray_random_x, ray_start_y, ray_random_y, rect_x1, rect_x2, rect_y1, rect_y2):


    #Funktion aufstellen, wenn es sich um eine Funktion handelt

    m = 0
    b = 0

    if ray_random_x != ray_start_x:


        #Funktion aus 2 Punkten aufstellen
        m = (ray_start_y-ray_random_y)/(ray_start_x-ray_random_x)
        b = ray_start_y-(m*ray_start_x)




    
    #-1 == Strahl geht in Richtung negativ
    # 0 == Strahlkoordinate bleibt konstant
    # 1 == Strahl geht in Richtung positiv

    ray_x_increase = 0
    
    if ray_start_x < ray_random_x:
        ray_x_increase = 1
    elif ray_start_x > ray_random_x:
        ray_x_increase = -1

    ray_y_increase = 0

    if ray_start_y < ray_random_y:
        ray_y_increase = 1
    elif ray_start_y > ray_random_y:
        ray_y_increase = -1
 
    
    distance_horizontal_upper = 10_000
    distance_horizontal_lower = 10_0000
    distance_vertical_right = 10_000
    distance_vertical_left = 10_000

#=======================================================================================================================
    #get distance_horizontal_upper

    #y Koordinate der oberen horizontalen Seite
    y_horizontal = np.maximum(rect_y1,rect_y2)

    y_direction_from_ray_start_y = 0

    if ray_start_y < y_horizontal:
        y_direction_from_ray_start_y = 1
    elif ray_start_y > y_horizontal:
        y_direction_from_ray_start_y = -1



    #--> welche x Koordinate hat der Strahl bei P(x, y_horizontal) ?

    #wenn keine Funktion:
    if ray_start_x == ray_random_x:
        
        #ray == vertikale Linie --> gleicher x Wert für alle y-Werte
        
        # intersection? (x Wert des Strahls innerhalb der x Werte der Ecken der Seite des rects und "same direction")
        if ray_start_x >= np.minimum(rect_x1, rect_x2) and ray_start_x <= np.maximum(rect_x1, rect_x2) and y_direction_from_ray_start_y == ray_y_increase:

            distance_horizontal_upper = np.abs(y_horizontal-ray_start_y)
            

    
    #Funktion
    else:
       
        #Anstieg und y-Achsenabschnitt wurden schon vorher berechnet

        #bei welchem x-Wert hat die Funktion die y Höhe von der Seite des Rects?

        

        x = (y_horizontal-b)/m



        if x >= np.minimum(rect_x1, rect_x2) and x <= np.maximum(rect_x1, rect_x2) and y_direction_from_ray_start_y == ray_y_increase and m != 0: 

            alpha = np.arctan(m) #Anstiegswinkel


                                                #Gegenkathete = y_horizontal - ray_start_y --> Sinus = GK/H nach H umstellen --> Länge der distance
            distance_horizontal_upper = (np.abs(y_horizontal-ray_start_y)/np.sin(alpha))
   
#=======================================================================================================================

#=======================================================================================================================
    #get distance_horizontal_lower

    #y Koordinate der oberen horizontalen Seite
    y_horizontal = np.minimum(rect_y1,rect_y2)

    y_direction_from_ray_start_y = 0

    if ray_start_y < y_horizontal:
        y_direction_from_ray_start_y = 1
    elif ray_start_y > y_horizontal:
        y_direction_from_ray_start_y = -1



    #--> welche x Koordinate hat der Strahl bei P(x, y_horizontal) ?

    #wenn keine Funktion:
    if ray_start_x == ray_random_x:
        
        #ray == vertikale Linie --> gleicher x Wert für alle y-Werte
        
        # intersection? (x Wert des Strahls innerhalb der x Werte der Ecken der Seite des rects und "same direction")
        if ray_start_x >= np.minimum(rect_x1, rect_x2) and ray_start_x <= np.maximum(rect_x1, rect_x2) and y_direction_from_ray_start_y == ray_y_increase:

            distance_horizontal_lower = np.abs(y_horizontal-ray_start_y)
            

    
    #Funktion
    else:

        #Anstieg und y-Achsenabschnitt wurden schon vorher berechnet

        #bei welchem x-Wert hat die Funktion die y Höhe von der Seite des Rects?

        x = (y_horizontal-b)/m



        if x >= np.minimum(rect_x1, rect_x2) and x <= np.maximum(rect_x1, rect_x2) and y_direction_from_ray_start_y == ray_y_increase and m != 0:

            alpha = np.arctan(m) #Anstiegswinkel


                                                #Gegenkathete = y_horizontal - ray_start_y --> Sinus = GK/H nach H umstellen --> Länge der distance
            distance_horizontal_lower = (np.abs(y_horizontal-ray_start_y)/np.sin(alpha))
   
#=======================================================================================================================

#=======================================================================================================================
#get distance_vertical_left

    x_vertical = np.minimum(rect_x1, rect_x2)

    x_direction_from_ray_start_x = 0

    if ray_start_x < x_vertical:
        x_direction_from_ray_start_x = 1
    elif ray_start_x > x_vertical:
        x_direction_from_ray_start_x = -1

    #--> welche y Koordinate hat der Strahl bei P(x_vertical, y) ?

    #wenn keine Funktion:
    if ray_start_x == ray_random_x:
        
        pass #muss nicht beachtet werden, weil diese Intersectionpunkte, die in Frage kommen, schon bei den Horizontalen dabei sind
            

    
    #Funktion
    else:

        #Anstieg und y-Achsenabschnitt wurden schon vorher berechnet

        #welchen y- Wert hat die Funktion beim Punkt(x_vertical, y)

        y = m*x_vertical+b




        if y >= np.minimum(rect_y1, rect_y2) and y <= np.maximum(rect_y1, rect_y2) and x_direction_from_ray_start_x == ray_x_increase:

            alpha = np.arctan(m) #Anstiegswinkel

            if m == 0:
                distance_vertical_left = np.abs(ray_start_x-x_vertical)
            
            else:
                
                distance_vertical_left = (np.abs(y-ray_start_y)/np.sin(alpha))
   
#=======================================================================================================================

#=======================================================================================================================
#get distance_vertical_right

    x_vertical = np.maximum(rect_x1, rect_x2)

    x_direction_from_ray_start_x = 0

    if ray_start_x < x_vertical:
        x_direction_from_ray_start_x = 1
    elif ray_start_x > x_vertical:
        x_direction_from_ray_start_x = -1

    #--> welche y Koordinate hat der Strahl bei P(x_vertical, y) ?

    #wenn keine Funktion:
    if ray_start_x == ray_random_x:
        
        pass #muss nicht beachtet werden, weil diese Intersectionpunkte, die in Frage kommen, schon bei den Horizontalen dabei sind
            

    
    #Funktion
    else:

        #Anstieg und y-Achsenabschnitt wurden schon vorher berechnet

        #welchen y- Wert hat die Funktion beim Punkt(x_vertical, y)

        y = m*x_vertical+b




        if y >= np.minimum(rect_y1, rect_y2) and y <= np.maximum(rect_y1, rect_y2) and x_direction_from_ray_start_x == ray_x_increase :

            alpha = np.arctan(m) #Anstiegswinkel


            if m == 0:
                distance_vertical_right = np.abs(ray_start_x-x_vertical)
            
            else:
                 distance_vertical_right = (np.abs(y-ray_start_y)/np.sin(alpha))

   # print([distance_vertical_right, distance_vertical_left, distance_horizontal_upper, distance_horizontal_lower])
    return(np.min([np.abs(distance_vertical_right), np.abs(distance_vertical_left), np.abs(distance_horizontal_upper), np.abs(distance_horizontal_lower)]))

""" if __name__ == "__main__":
    

    r_x1 = 10
    r_x2 = 15
    r_y1 = 2
    r_y2 = 6

    ray_x_start = 2 
    ray_y_start = 6
    ray_x_rand = 12
    ray_y_rand = 6

    intersection(ray_start_x= ray_x_start, ray_random_x= ray_x_rand, ray_start_y= ray_y_start, ray_random_y= ray_y_rand, rect_x1= r_x1, rect_x2 = r_x2, rect_y1 = r_y1, rect_y2 = r_y2)



    left, bottom, width, height = (np.minimum(r_x1, r_x2), np.minimum(r_y1, r_y2), np.abs(r_x1 - r_x2), np.abs(r_y1- r_y2))
    rect = plt.Rectangle((left, bottom), width, height,
                        facecolor="black", alpha=0.1)

    fig, ax = plt.subplots()
    ax.add_patch(rect)

    bbox = Bbox.from_bounds(left, bottom, width, height)

    vertices = np.zeros(shape = (2,2))
    vertices[0][0] = ray_x_start
    vertices[0][1] = ray_y_start
    vertices[1][0] = ray_x_rand
    vertices[1][1] = ray_y_rand
    
    path = Path(vertices)
    if path.intersects_bbox(bbox):
        color = 'r'
    else:
        color = 'b'
    ax.plot(vertices[:, 0], vertices[:, 1], color=color)

    plt.show() """


#unit tests wurden für den fall der intersection weitestgehend gemacht, ich muss noch paar machen für den fall, dass es keine intersection gibt