import pygame
import serial
import time

PORT = "/dev/cu.usbmodem1101"  # change if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

pygame.init()
screen = pygame.display.set_mode((300, 150))
pygame.display.set_caption("Motor Keyboard Control")

last_cmd = None

running = True
while running:
    pygame.event.pump()

    keys = pygame.key.get_pressed()

    if keys[pygame.K_1]:
        cmd = b"1"   # backward
    elif keys[pygame.K_2]:
        cmd = b"2"   # forward
    else:
        cmd = b"0"   # stop

    if cmd != last_cmd:
        ser.write(cmd)
        last_cmd = cmd

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    time.sleep(0.03)

ser.write(b"0")
ser.close()
pygame.quit()