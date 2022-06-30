import pygame
import random
import numpy as np
from util import set_vector_magnitude
from params import *


class Boid(pygame.sprite.Sprite):
    image = None
    flock = pygame.sprite.Group()

    def __init__(self, position=None):
        pygame.sprite.Sprite.__init__(self, self.flock)
        self.id = len(self.flock)
        self.rect = self.image.get_rect(center=(4, 13))
        x = random.randint(0, WINDOW_WIDTH)
        y = random.randint(0, WINDOW_HEIGHT)
        self.position = position if position is not None else np.array([x, y])
        self.velocity = set_vector_magnitude(np.random.rand(2), 4)  # set vector mag to 4 (higher speed)
        self.acceleration = np.zeros(2)

    @staticmethod
    def get_angle(v1, v2):
        v1_unit = v1 / np.linalg.norm(v1)
        v2_unit = v2 / np.linalg.norm(v2)
        dot_prod = np.dot(v1_unit, v2_unit)
        angle = np.arccos(dot_prod)
        angle = abs(np.rad2deg(angle))
        return angle

    def update(self):
        self.move()
        # reset acceleration to prevent accumulation over time
        self.acceleration = np.zeros(2)
        self.rect = self.image.get_rect(center=self.rect.center)
        self.rect.center = (self.position[0], self.position[1])

    def move(self):
        #TODO
        separation = self.separation()
        cohesion = self.cohesion()
        alignment = self.alignment()
        # force accumulation - acceleration is the sum of all forces acting on an object
        force_sum = np.add.reduce([separation, cohesion, alignment])
        # applying force to object is equal to setting acceleration to that force
        self.acceleration = force_sum

        self.check_borders()
        self.position = np.add(self.position, self.velocity)
        self.velocity = np.add(self.velocity, self.acceleration)
        # limit speed
        self.velocity = set_vector_magnitude(self.velocity, BOID_MAX_SPEED)


    def helper(self, is_cohesion, steering, factor, neighbours_number):
        for other_boid in self.flock:
            distance_between_boids = np.linalg.norm(np.subtract(self.position, other_boid.position))
            if other_boid is not self and distance_between_boids < BOID_PERCEPTION_DISTANCE and self.get_angle(self.velocity, np.subtract(self.position, other_boid.position)) < BOID_MAX_ANGLE:
                if(is_cohesion == 1):
                    steering = np.add(steering, other_boid.position)
                else:
                    steering = np.add(steering, other_boid.velocity)
                neighbours_number += 1
        if neighbours_number > 0:
            steering = steering / neighbours_number
            if(is_cohesion == 1):
                steering = np.subtract(steering, self.position)
            steering = set_vector_magnitude(steering, BOID_MAX_SPEED)
            steering = np.subtract(steering, self.velocity)
            steering = set_vector_magnitude(steering, factor)
        return steering

    # Cohesion rule - align boid's position with **neighbouring** agents
    # Returns the force which needs to be applied to the agent to correct agent's 'course of movement'.
    def cohesion(self):
        steering = np.zeros(2)  # force which needs to be applied to boid to 'correct' it's current moving direction

        # TODO
        neighbours_number = 0
        #for other_boid in self.flock:
            #distance_between_boids = np.linalg.norm(np.subtract(self.position, other_boid.position))
            #if other_boid is not self and distance_between_boids < BOID_PERCEPTION_DISTANCE and self.get_angle(self.velocity, np.subtract(self.position, other_boid.position)) < BOID_MAX_ANGLE:
                #steering = np.add(steering, other_boid.position)
                #neighbours_number += 1
        #if neighbours_number > 0:
            #steering = steering / neighbours_number
            #steering = np.subtract(steering, self.position)
            #steering = set_vector_magnitude(steering, BOID_MAX_SPEED)
            #steering = np.subtract(steering, self.velocity)
            #steering = set_vector_magnitude(steering, BOID_COHESION_FACTOR)
        steering = self.helper(1, steering, BOID_COHESION_FACTOR, neighbours_number)
        return steering

    # Alignment rule - align boid's orientation/velocity with **neighbouring** agents
    # Returns the force which needs to be applied to the agent to correct it's 'course of movement'.
    def alignment(self):
        steering = np.zeros(2)  # force which needs to be applied to boid to 'correct' it's current moving direction

        # TODO
        neighbours_number = 0
        #for other_boid in self.flock:
            #distance_between_boids = np.linalg.norm(np.subtract(self.position, other_boid.position))
            #if other_boid is not self and distance_between_boids < BOID_PERCEPTION_DISTANCE and self.get_angle(self.velocity, np.subtract(self.position, other_boid.position)) < BOID_MAX_ANGLE:
                #steering = np.add(steering, other_boid.velocity)
                #neighbours_number += 1
        #if neighbours_number > 0:
            #steering = steering / neighbours_number
            #steering = set_vector_magnitude(steering, BOID_MAX_SPEED)
            #steering = np.subtract(steering, self.velocity)
            #steering = set_vector_magnitude(steering, BOID_ALIGNMENT_FACTOR)
        steering = self.helper(0, steering, BOID_ALIGNMENT_FACTOR, neighbours_number)
        return steering

    # Separation rule - separate boid to avoid crowds
    # Returns the force which needs to be applied to the agent to correct agent's 'course of movement'.
    def separation(self):
        steering = np.zeros(2)  # force which needs to be applied to boid to 'correct' it's current moving direction
        neighbours_number = 0
        for other_boid in self.flock:
            distance_between_boids = np.linalg.norm(np.subtract(self.position, other_boid.position))
            # find all boids within sight
            if other_boid is not self and distance_between_boids < BOID_PERCEPTION_DISTANCE and self.get_angle(self.velocity, np.subtract(self.position, other_boid.position)) < BOID_MAX_ANGLE:
                # vector in opposite direction
                difference = np.subtract(self.position, other_boid.position)
                # inversely proportional to distance
                steering = np.add(steering, difference / distance_between_boids)
                neighbours_number += 1

        # calculate average and find force vector
        if neighbours_number > 0:
            steering = steering/neighbours_number
            # to increase speed, instead of averaging velocity (just average direction)
            steering = set_vector_magnitude(steering, BOID_MAX_SPEED)
            # subtract vectors to find vector which will correct the course of the agent
            steering = np.subtract(steering, self.velocity)
            # limit force vector to get consistent magnitude across agents
            steering = set_vector_magnitude(steering, BOID_SEPARATION_FACTOR)

        return steering

    def check_borders(self):
        # Repel boids when near border of screen
        if abs(WINDOW_WIDTH / 2 - self.position[0]) > WINDOW_WIDTH / 2 - BORDER_WIDTH:
            repel = BORDER_REPEL_FACTOR if self.position[0] < WINDOW_WIDTH / 2 else -BORDER_REPEL_FACTOR
            self.velocity = np.array([self.velocity[0] + repel, self.velocity[1]])
        if abs(WINDOW_HEIGHT / 2 - self.position[1]) > WINDOW_HEIGHT / 2 - BORDER_WIDTH:
            repel = BORDER_REPEL_FACTOR if self.position[1] < WINDOW_HEIGHT / 2 else -BORDER_REPEL_FACTOR
            self.velocity = np.array([self.velocity[0], self.velocity[1] + repel])
