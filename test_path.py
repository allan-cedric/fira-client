#!/usr/bin/env python3
import cv2 as cv
import numpy as np

negated = True

w = 300
h = 300

image = np.zeros((w, h, 3), dtype=np.uint8)
image.fill(255 * negated)


while True:
    command = input()

    if command: 

        if command[0] == "c":
            _, x, y, radius = command.split()
            x = int(float(x))
            y = int(float(y))
            radius = int(float(radius))
            cv.circle(image, (x, h - y), radius, (0, 0, 255), 3)

        elif command[0] == "l":
            _, p1x, p1y, p2x, p2y = command.split()
            p1x = int(float(p1x))
            p1y = int(float(p1y))
            p2x = int(float(p2x))
            p2y = int(float(p2y))
            cv.line(image, (p1x, h - p1y), (p2x, h - p2y), (255, 0, 0), 3)
    
    cv.imshow("output", image)
    cv.waitKey(0)