import sys

import pygame
from pygame.locals import *
import os.path
from params import *
from boid import Boid
from util import load_image


main_dir = os.path.split(os.path.abspath(__file__))[0]


def init_pygame():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('Boids')
    return screen


def check_events():
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            sys.exit()


def init_flock():
    # A container to hold and manage multiple visible game objects
    flock = pygame.sprite.Group()
    Boid.image = load_image('boid.png')
    Boid.flock = flock
    # Create boids
    for _ in range(BOIDS_NUMBER):
        Boid()
    return flock


def draw_env(screen, flock):
    pygame.time.Clock().tick(100)
    screen.fill([0, 0, 0])
    flock.update()
    flock.draw(screen)
    pygame.display.update()


def main():
    screen = init_pygame()
    flock = init_flock()

    while True:
        check_events()
        draw_env(screen, flock)


main()
