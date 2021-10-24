import numpy as np
import pymunk
import pygame


class Garbage:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 20
        self.height = 20
        self.body = pymunk.Body()
       

        self.body.position = self.x, self.y
      

        self.shape = pymunk.Poly.create_box(self.body, (self.width, self.height), 0.0) 
        self.shape.mass = 1
        self.shape.friction = 0.7
        self.shape.color = pygame.Color("black")
        
       

    def ground_friction(self):
        
        self.body.velocity = self.body.velocity[0]/2, self.body.velocity[1]/2
        self.body.angular_velocity = self.body.angular_velocity/2

  