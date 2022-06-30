import pygame
import os.path
import math

main_dir = os.path.split(os.path.abspath(__file__))[0]


def load_image(file):
    file = os.path.join(main_dir, file)
    try:
        image = pygame.image.load(file)
    except pygame.error:
        raise SystemExit('Could not load image')
    return image.convert_alpha()


def set_vector_magnitude(vector, mag):
    vx, vy = vector
    n = math.sqrt(vx**2 + vy**2)
    f = mag / n
    return [f * vx, f * vy]
