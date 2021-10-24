import numpy as np
import pygame
import pymunk
import collections
import random
from ray_aabb_intersection_agorithm_2 import intersection

class Agent:
    


  


    def __init__(self):
        
        self.x = 300
        self.y = 750
     
        self.body = pymunk.Body()
        self.body.position = self.x, self.y
       

        self.shape_left = pymunk.Poly(self.body, [[35.5-50,36.5-62.5],[40.5-50,36.5-62.5],[27.5-50,36.5-35.5],[22.5-50,36.5-35.5]])
        self.shape_right = pymunk.Poly(self.body, [[59.5-50,36.5-62.5],[64.5-50,36.5-62.5],[77.5-50,36.5-35.5],[72.5-50,36.5-35.5]])
        self.shape_mid = pymunk.Poly(self.body, [[35.5-50,36.5-99.5],[64.5-50,36.5-99.5],[64.5-50,36.5-62.5],[35.5-50,36.5-62.5]])

        self.shape_left.color = pygame.Color("black")
        self.shape_right.color = pygame.Color("black")
        self.shape_mid.color = pygame.Color("white")



        self.shape_left.mass = 10
        self.shape_left.friction = 0.7
        self.shape_right.mass = 10
        self.shape_right.friction = 0.7
        self.shape_mid.mass = 10
        self.shape_mid.friction = 0.7


        self.speed_forward = 80
        self.speed_backward = 40
        self.delta_angle = 0.06

        self.body.angle = np.pi #um 180° drehen

        self.num_sensors = 6   #15
        self.d_angle = ((360/self.num_sensors)/180) * np.pi  #delta winkel im bogenmaß wenn Sensoren im gleichen Abstand verteilt werden sollen
        self.go_to_garbage = True


    def move(self, visible_garbage_vecs, visible_trashcan_vecs):
       angle = 0.5*np.pi - self.body.angle
       self.body.velocity = (0,0)
       self.body.angular_velocity = 0

       if self.go_to_garbage: # zum Müll fahren
           
           if len(visible_garbage_vecs) >0: #Müll sichtbar -->#ausrichten und hinfahren 

               #closest garbage
               min_distance = 10000000000000
               min_distance_idx = -1

               for i in range(0, len(visible_garbage_vecs)):

                   distance = visible_garbage_vecs[i][0]**2 + visible_garbage_vecs[i][1]**2
                   if distance < min_distance:
                       min_distance = distance
                       min_distance_idx = i

               closest_garbage_x = visible_garbage_vecs[min_distance_idx][0]
               closest_garbage_y = visible_garbage_vecs[min_distance_idx][1]

               
               self.body.velocity = (-(self.speed_forward*np.cos(angle)),  (self.speed_forward*np.sin(angle)))

               if (closest_garbage_x > 15 or closest_garbage_x < 1-5) and closest_garbage_y < 40:   #if garbage too close and agent cant rotate to it: drive backwards
                    self.body.velocity = (-(self.speed_backward*(- np.cos(angle))), self.speed_backward*(- np.sin(angle)))

               elif closest_garbage_x > 0  and not(closest_garbage_x < 10 and  closest_garbage_x > -10): #try to minimize the angle between garbage and agent
                     self.body.angular_velocity += 1.5
               elif closest_garbage_x < 0  and not(closest_garbage_x < 10 and  closest_garbage_x > -10): #try to minimize the angle between garbage and agent
                     self.body.angular_velocity -= 1.5
                
               

               if closest_garbage_x < 10 and  closest_garbage_x > -10 and closest_garbage_y<40: #garbage ist in den armen
                   self.go_to_garbage = False 

               
           else:    #Müll nicht sichtbar
                   self.body.velocity = (-(self.speed_backward*(- np.cos(angle))), self.speed_backward*(- np.sin(angle))) #drive backwards and rotate

                   self.body.angular_velocity += 1 #suchen nach Müll


       else:    #zum Mülleimer fahren
            if len(visible_trashcan_vecs) >0: #Mülleimer sichtbar -->  #ausrichten und hinfahren
                
               trashcan_x = visible_trashcan_vecs[0][0]
               trashcan_y = visible_trashcan_vecs[0][1]

              
               self.body.velocity = (-(self.speed_forward*np.cos(angle)),  (self.speed_forward*np.sin(angle)))

               if trashcan_x > 0  and not(trashcan_x < 10 and  trashcan_x > -10):   #try to minimize the angle between trashcan and agent
                     self.body.angular_velocity += 1.5
               elif trashcan_x < 0  and not(trashcan_x < 10 and  trashcan_x > -10): #try to minimize the angle between trashcan and agent
                     self.body.angular_velocity -= 1.5

               if trashcan_y < 40: #Müll weggebracht
                   self.go_to_garbage = True



            else: #Mülleimer nicht sichtbar
                  #suchen nach Mülleimer

                   self.body.velocity = (-(self.speed_forward*np.cos(angle)),  (self.speed_forward*np.sin(angle))) #drive forwards and rotate 

                   self.body.angular_velocity += 1



    #diese Methode verändert den Winkel des Laserstrahls/der Laserstrahlen so, dass sie nicht mehr im gleichen Abstand voneinander sind, sondern effektiver platziert sind
    def modify_laser_angle(self, angle_to_modify):
        
        if angle_to_modify == 0:

            return angle_to_modify + ((360/15)/180) * np.pi 

        elif angle_to_modify == ((360/15)/180) * np.pi:

            return np.pi/2

        elif angle_to_modify == np.pi/2:

            return np.pi

        elif angle_to_modify == np.pi:

            return 1.5 * np.pi

        elif angle_to_modify == 1.5*np.pi:

            return 0- ((360/15)/180) * np.pi 

       
        return 0





        

    def sensor_input(self, obstacles):
        
        #immer den kürzesten Abstand zum Obstacle in das Array eintragen

        angle = 0

        sensor_values = np.ones(shape = (1,self.num_sensors))  #shape = (batch_size, num_sensors)

        for i in range(0,self.num_sensors):
            sensor_values[0][i] = 10_000 #distance auf 10_000 setzen, damit sie überschrieben werden kann (nur kleinere Werte als die, die schon drinne sind, können rein)


        for i in range(0,self.num_sensors):

           
               

                for j in range(0, len(obstacles)):
                
                    #in die robotermitte geshiftete "random" (eig ja nicht random ("*100")) punkte
                    distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 =  obstacles[j].body.position[0]-obstacles[j].width/2, rect_x2 =obstacles[j].body.position[0]+obstacles[j].width/2, rect_y1 = obstacles[j].body.position[1]-obstacles[j].height/2, rect_y2= obstacles[j].body.position[1]+obstacles[j].height/2 )
  
                    if distance < sensor_values[0][i]:
                        sensor_values[0][i] = distance

                #walls zu rects machen und dann die gleiche intersection funktion nutzen (könnte man später nat noch optimieren, indem man nur strecke und strahl intersection bei der relevanten seite macht)
                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 10, rect_y1 = 0, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 590 , rect_x2 = 600, rect_y1 = 0, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 600, rect_y1 = 0, rect_y2 = 10)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 600, rect_y1 = 790, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

               

                angle = angle + self.d_angle

        return sensor_values  

   
   


    def smart_sensor_input(self, obstacles):


                   #immer den kürzesten Abstand zum Obstacle in das Array eintragen

        angle = 0

        sensor_values = np.ones(shape = (1,self.num_sensors))  #shape = (batch_size, num_sensors)

        for i in range(0,self.num_sensors):
            sensor_values[0][i] = 10_000 #distance auf 10_000 setzen, damit sie überschrieben werden kann (nur kleinere Werte als die, die schon drinne sind, können rein)


        for i in range(0,self.num_sensors):

           
               

                for j in range(0, len(obstacles)):
                
                    #in die robotermitte geshiftete "random" (eig ja nicht random ("*100")) punkte
                    distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 =  obstacles[j].body.position[0]-obstacles[j].width/2, rect_x2 =obstacles[j].body.position[0]+obstacles[j].width/2, rect_y1 = obstacles[j].body.position[1]-obstacles[j].height/2, rect_y2= obstacles[j].body.position[1]+obstacles[j].height/2 )
  
                    if distance < sensor_values[0][i]:
                        sensor_values[0][i] = distance

                #walls zu rects machen und dann die gleiche intersection funktion nutzen (könnte man später nat noch optimieren, indem man nur strecke und strahl intersection bei der relevanten seite macht)
                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 10, rect_y1 = 0, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 590 , rect_x2 = 600, rect_y1 = 0, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 600, rect_y1 = 0, rect_y2 = 10)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

                distance = intersection(ray_start_x = (self.body.position[0]+np.sin(self.body.angle)*45), ray_random_x= (self.body.position[0]+np.sin(self.body.angle)*45)+np.sin(angle-self.body.angle) * 100, ray_start_y=  (self.body.position[1]-np.cos(self.body.angle)*45), ray_random_y= (self.body.position[1]-np.cos(self.body.angle)*45)+np.cos(angle-self.body.angle)*100, rect_x1 = 0 , rect_x2 = 600, rect_y1 = 790, rect_y2 = 800)
                
                if distance < sensor_values[0][i]:
                    sensor_values[0][i] = distance

               

                angle = self.modify_laser_angle(angle)

        return sensor_values  

   






            
        




