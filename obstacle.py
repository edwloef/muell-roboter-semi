import numpy as np
import pymunk
import pygame

class Obstacle:

    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.width = np.random.randint(20,40)
        self.height = np.random.randint(20,40)

        self.body = pymunk.Body(body_type= pymunk.Body.STATIC)
        self.body.position = (self.x, self.y)
        self.shape = pymunk.Poly(self.body, [[-self.width/2,-self.height/2], [self.width/2,-self.height/2], [self.width/2, self.height/2], [-self.width/2,self.height/2]], radius= 1)

        self.shape.color = pygame.Color("red")
        
        




