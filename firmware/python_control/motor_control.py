import pygame
import serial
import time

PORT = "/dev/cu.usbmodem1101"
BAUD = 115200

ser = serial.Serial(PORT, BAUD)
time.sleep(2)

pygame.init()

screen = pygame.display.set_mode((400, 200))
pygame.display.set_caption("Motor Control")

running = True
last_cmd = None

while running:

    pygame.event.pump()

    keys = pygame.key.get_pressed()

    if keys[pygame.K_1]:
        cmd = b'1'

    elif keys[pygame.K_2]:
        cmd = b'2'

    else:
        cmd = b'0'

    if cmd != last_cmd:
        ser.write(cmd)
        print("Sent:", cmd)
        last_cmd = cmd

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    time.sleep(0.02)

ser.write(b'0')
ser.close()
pygame.quit()