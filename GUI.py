

import pygame
import numpy as np
import sys
import os

from agent import Agent

from obstacle import Obstacle
from garbage import Garbage
import pymunk
import pymunk.pygame_util




def point_in_aabb(point_x, point_y, rect_x1, rect_x2, rect_y1, rect_y2):
        return  (point_x > np.minimum(rect_x1, rect_x2)) and (point_x < np.maximum(rect_x1, rect_x2)) and (point_y > np.minimum(rect_y1, rect_y2)) and (point_y < np.maximum(rect_y1, rect_y2)) 



def get_local_vector_3(agent_x, agent_y, agent_angle, point_x, point_y):

       #Vorgehen:
       #Roboter auf 0 Grad (bzw. eigentlich 180, weil der Roboter ja falsch herum benutzt wird) rotieren
       #Punkt um den Mittelpunkt des Roboters um den gleichen Winkel, um den der Roboter vorher rotiert wurde, rotieren
       #nun kann mit dem "normalen" (und keinem lokalen Koordinatensystem des Roboters) gearbeitet werden und der "lokale" Vektor einfach bestimmt werden
 
       #Umsetzung der Rotation:
       #Punkt, um den rotiert werden soll (agent_x, agent_y), zum Ursprung (0,0) shiften --> Punkt, der rotiert werden soll, wird mitgeshiftet
       #Punkt, der rotiert werden soll, mit der Drehmatrix rotieren
       #Punkt, der rotiert wurde, zurück shiften

       #origin = (agent_x, agent_y)

       agent_angle = agent_angle - np.pi #der Roboter wurde ja beim spawnen um 180 Grad gedreht, also sind die "0 Grad" (Winkel zur Senkrechten) nocht 0, sondern 180 Grad --> muss natürlich hier wieder zu 0 Gemacht werden (geht ja um die absolute WinkelGRÖßE)
       
       rotation_angle =  (-1)* agent_angle  # *(-1) weil das Ding IM UHRZEIGERSINN wächst und nicht gegen den Uhrzeigersinn, wie die Drehmatrix gedacht ist
                                           

       shift_x = agent_x
       shift_y = agent_y

       point_x = point_x - shift_x
       point_y = point_y - shift_y

       point_x_ = point_x * np.cos(rotation_angle) - point_y * np.sin(rotation_angle)
       point_y_ = point_x * np.sin(rotation_angle) + point_y * np.cos(rotation_angle)

       point_x_ = point_x_ + shift_x
       point_y_ = point_y_ + shift_y

       loc_vec_x = point_x_ - agent_x
       loc_vec_y =  point_y_ - agent_y 


       return (loc_vec_x, loc_vec_y*(-1)) # *(-1) weil das Koordinatensystem bei der y Achse ja andersrum ist






NUM_OBSTACLES = 0
NUM_GARBAGE = 5

draw_on_screen = True




#pymunk stuff
space = pymunk.Space()
space.iterations = 10
space.sleep_time_threshold = 0.5



#walls/borders
#left wall
left_wall_body = pymunk.Body(body_type=pymunk.Body.STATIC)
left_wall_shape = pymunk.Segment(left_wall_body, (0,0), (0,800), 10)
left_wall_shape.color = pygame.Color("grey")
left_wall_shape.elasticity = 1
left_wall_shape.friction = 1
space.add(left_wall_body, left_wall_shape)

#right wall
right_wall_body = pymunk.Body(body_type=pymunk.Body.STATIC)
right_wall_shape = pymunk.Segment(right_wall_body, (600,0), (600,800), 10)
right_wall_shape.color = pygame.Color("grey")
right_wall_shape.elasticity = 1
right_wall_shape.friction = 1
space.add(right_wall_body, right_wall_shape)

#upper wall
upper_wall_body = pymunk.Body(body_type=pymunk.Body.STATIC)
upper_wall_shape = pymunk.Segment(upper_wall_body, (0,800), (600,800), 10)
upper_wall_shape.color = pygame.Color("grey")
upper_wall_shape.elasticity = 1
upper_wall_shape.friction = 1
space.add(upper_wall_body, upper_wall_shape)

#lower wall
lower_wall_body = pymunk.Body(body_type=pymunk.Body.STATIC)
lower_wall_shape = pymunk.Segment(lower_wall_body, (0,0), (600,0), 10)
lower_wall_shape.color = pygame.Color("grey")
lower_wall_shape.elasticity = 1
lower_wall_shape.friction = 1
space.add(lower_wall_body, lower_wall_shape)




agent = Agent()
space.add(agent.body, agent.shape_mid,agent.shape_left, agent.shape_right)


#pygame stuff
pygame.init()
frame = pygame.display.set_mode([1000,800])
clock = pygame.time.Clock()

options = pymunk.pygame_util.DrawOptions(frame)




obstacles = []
garbage = []



#generate obstacles
for f in range(0, int(NUM_OBSTACLES/2)):


       obstacles.append(Obstacle(200, f*200 + 200))

       obstacles.append(Obstacle(400, f*200 + 200))



       space.add(obstacles[f*2].body, obstacles[f*2].shape)
       space.add(obstacles[f*2+1].body, obstacles[f*2+1].shape)



#generate garbage
for f in range(0, NUM_GARBAGE):



       garbage.append(Garbage(np.random.randint(200,400), np.random.randint(200,600)))
       space.add(garbage[f].body, garbage[f].shape)


#generate trash can position
trash_can_position_x1 = np.random.randint(0, 531)
trash_can_position_x2 = trash_can_position_x1 + 70
trash_can_position_y1 = 0
top_or_bottom = np.random.randint(0,2)
if top_or_bottom == 0:
       trash_can_position_y1 = 730
trash_can_position_y2 = trash_can_position_y1 + 70
trash_can_midpoint = (trash_can_position_x1 + (trash_can_position_x2-trash_can_position_x1)/2,  trash_can_position_y1 + (trash_can_position_y2-trash_can_position_y1)/2)




       #game
while True:
            
              

              for event in pygame.event.get():
                     if event.type == pygame.QUIT:
                     
                             sys.exit()

              
           #   pressed = pygame.key.get_pressed()

          

           #   if pressed[pygame.K_h]:
               #      draw_on_screen = not draw_on_screen
          

              
           
              frame.fill((100,100,100))
         
                     
              #draw trash can
              pygame.draw.rect(frame, (255,255,0),(trash_can_position_x1, trash_can_position_y1, trash_can_position_x2 - trash_can_position_x1,trash_can_position_y2-trash_can_position_y1))
              

              #draw sensor
              angle = 0

            
                     #Wenn Sensoren im gleichen Abstand sein sollen
            # for i in range(0,agent.num_sensors):

                        #    pygame.draw.line(frame, (50,255,50), (agent.body.position[0]+np.sin(agent.body.angle)*45, agent.body.position[1]-np.cos(agent.body.angle)*45), ((agent.body.position[0]+np.sin(agent.body.angle)*45)+np.sin(angle-agent.body.angle) *800, (agent.body.position[1]-np.cos(agent.body.angle)*45)+np.cos(angle-agent.body.angle)*800))

                      #      angle = angle + agent.d_angle

                     #Wenn die Sensoren im schlauen Abstand sein sollen
              for i in range(0, agent.num_sensors):
                            
                            pygame.draw.line(frame, (50,255,50), (agent.body.position[0]+np.sin(agent.body.angle)*45, agent.body.position[1]-np.cos(agent.body.angle)*45), ((agent.body.position[0]+np.sin(agent.body.angle)*45)+np.sin(angle-agent.body.angle) *800, (agent.body.position[1]-np.cos(agent.body.angle)*45)+np.cos(angle-agent.body.angle)*800))

                            angle = agent.modify_laser_angle(angle)


              for i in range(0, len(garbage)):
                     vec =   get_local_vector_3(agent.body.position[0]+np.sin(agent.body.angle)*45,  agent.body.position[1]-np.cos(agent.body.angle)*45, agent.body.angle, garbage[i].body.position[0], garbage[i].body.position[1])
                     if vec[1] > 0:
                            garbage[i].shape.color = pygame.Color("blue")    #er soll nur die Mülle sehen, die vor ihm sind (nicht hinter ihm)
                         
                     else:
                             garbage[i].shape.color = pygame.Color("black")
          
             
                                           
            #  vec_to_garbage = get_local_vector_3(agent.body.position[0]+np.sin(agent.body.angle)*45,  agent.body.position[1]-np.cos(agent.body.angle)*45, agent.body.angle, garbage[0].body.position[0], garbage[0].body.position[1])
             # vec_to_trash_can = get_local_vector_3(agent.body.position[0]+np.sin(agent.body.angle)*45,  agent.body.position[1]-np.cos(agent.body.angle)*45, agent.body.angle,trash_can_midpoint[0], trash_can_midpoint[1])   #trash can mid point
              
              vecs = []
              for i in range(0,len(garbage)):
                  vec = get_local_vector_3(agent.body.position[0]+np.sin(agent.body.angle)*45,  agent.body.position[1]-np.cos(agent.body.angle)*45, agent.body.angle, garbage[i].body.position[0], garbage[i].body.position[1])
                  
                  if vec[1] > 0:
                   vecs.append(vec)
                  
              trashcans = [] #da kommt max 1 rein, aber damit man auch übergeben kann, dass man sie nicht sieht, mach ich das als liste (length 0 bedeutet nicht sichtbar)

              vec_to_trash_can = get_local_vector_3(agent.body.position[0]+np.sin(agent.body.angle)*45,  agent.body.position[1]-np.cos(agent.body.angle)*45, agent.body.angle,trash_can_midpoint[0], trash_can_midpoint[1])   #trash can mid point
              
              if vec_to_trash_can[1] > 0:
                     trashcans.append(vec_to_trash_can)


              agent.move(vecs, trashcans)
             
              #angle = np.pi/8
             # pygame.draw.polygon(frame, (50,255,50,250), [(agent.body.position[0]+np.sin(agent.body.angle)*45, agent.body.position[1]-np.cos(agent.body.angle)*45),((agent.body.position[0]+np.sin(agent.body.angle)*45)+np.sin(angle-agent.body.angle) *400, (agent.body.position[1]-np.cos(agent.body.angle)*45)+np.cos(angle-agent.body.angle)*400),((agent.body.position[0]+np.sin(agent.body.angle)*45)+np.sin(angle*(-1)-agent.body.angle) *400, (agent.body.position[1]-np.cos(agent.body.angle)*45)+np.cos(angle*(-1)-agent.body.angle)*400)],1)
             
           
             
              space.debug_draw(options)

              pygame.draw.rect(frame, (100,100,100),(600, 0, 400, 800))
              
              
            
              pygame.display.update()

              space.step(1/30)
              
            
              
             

           
              garbage_idx_to_remove = []

              for f in range(len(garbage)):

                            garbage[f].ground_friction()
                            #garbage in trash can?
                            if point_in_aabb(point_x = garbage[f].body.position[0], point_y= garbage[f].body.position[1], rect_x1= trash_can_position_x1-10, rect_x2= trash_can_position_x2-10, rect_y1= trash_can_position_y1-10, rect_y2 = trash_can_position_y2-10):
                                   garbage_idx_to_remove.append(f)

              for f in range(len(garbage_idx_to_remove)):
                            #remove trash that was brought to the trash can
                            space.remove(garbage[garbage_idx_to_remove[f]].body,garbage[garbage_idx_to_remove[f]].shape)
                            garbage.pop(garbage_idx_to_remove[f])
                            end = True



              clock.tick(30)

