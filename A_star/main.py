import pygame
from environment import *
import sys

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Wild Fire")

main(WIN, WIDTH)
sys.exit()