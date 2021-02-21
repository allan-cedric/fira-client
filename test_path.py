#!/usr/bin/env python3
import cv2 as cv
import numpy as np

negated = True

w = 1000
h = 1000
magnitude = 5

image = np.zeros((w, h, 3), dtype=np.uint8)
image.fill(255 * negated)


while True:
    command = input()
    print(command)

    if command: 

        if command[0] == "c":
            _, x, y, radius, index = command.split()
            x = int(float(x))
            y = int(float(y))
            radius = int(float(radius))
            if radius == 0:
                radius = 1
            cv.circle(image, (magnitude * x,  (h - magnitude *y)), radius * magnitude, (0, 0, 255), 2)
            cv.putText(image, index, (magnitude *x, (h - magnitude *y)), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))

        elif command[0] == "l":
            _, p1x, p1y, p2x, p2y = command.split()
            p1x = int(float(p1x))
            p1y = int(float(p1y))
            cv.circle(image, (magnitude * p1x,  (h - magnitude *p1y)), 5, (0, 255, 0), cv.FILLED)
            p2x = int(float(p2x))
            p2y = int(float(p2y))
            cv.circle(image, (magnitude * p2x,  (h - magnitude *p2y)), 5, (0, 255, 0), cv.FILLED)
            cv.line(image, (magnitude *p1x, (h - magnitude *p1y)), (magnitude *p2x,(h - magnitude *p2y)), (255, 0, 0), 3)
    
        elif command[0] == "e":
            _, p1x, p1y, p2x, p2y = command.split()
            p1x = int(float(p1x))
            p1y = int(float(p1y))
            cv.circle(image, (magnitude * p1x,  (h - magnitude *p1y)), 5, (0, 255, 255), cv.FILLED)
            p2x = int(float(p2x))
            p2y = int(float(p2y))
            cv.circle(image, (magnitude * p2x,  (h - magnitude *p2y)), 5, (0, 255, 255), cv.FILLED)
            cv.line(image, (magnitude *p1x, (h - magnitude *p1y)), (magnitude *p2x,(h - magnitude *p2y)), (255, 255, 0), 3)

    
    cv.imshow("output", image)
    cv.waitKey(0)