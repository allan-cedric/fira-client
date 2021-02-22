#!/usr/bin/env python3
import cv2 as cv
import numpy as np

class Edge:
    def __init__(self, x1, y1, x2, y2):
        self.p1x = x1
        self.p1y = y1
        self.p2x = x2
        self.p2y = y2

class Circle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

negated = True

w = 1000
h = 1000
magnitude = 5

image = np.zeros((w, h, 3), dtype=np.uint8)
image.fill(255 * negated)

circles = []
edges = []
path_edges = []


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
            
            c = Circle(x, y, radius)
            circles.append(c)
        
        if command[0] == "l":
            _, p1x, p1y, p2x, p2y = command.split()
            p1x = int(float(p1x))
            p1y = int(float(p1y))
            p2x = int(float(p2x))
            p2y = int(float(p2y))
            l = Edge(p1x, p1y, p2x, p2y)
            path_edges.append(l)


        if command[0] == "e":
            _, p1x, p1y, p2x, p2y = command.split()
            p1x = int(float(p1x))
            p1y = int(float(p1y))
            p2x = int(float(p2x))
            p2y = int(float(p2y))
            e = Edge(p1x, p1y, p2x, p2y)
            edges.append(e)


        if command[0] == "clear":
            circles.clear()
            edges.clear()
            path_edges.clear()
            image.fill(255 * negated)


    for line in path_edges:
        cv.circle(image, (magnitude * line.p1x,  (h - magnitude *line.p1y)), 5, (0, 255, 0), cv.FILLED)
        cv.circle(image, (magnitude * line.p2x,  (h - magnitude *line.p2y)), 5, (0, 255, 0), cv.FILLED)
        cv.line(image, (magnitude *line.p1x, (h - magnitude *line.p1y)), (magnitude *line.p2x,(h - magnitude *line.p2y)), (255, 0, 0), 3)

    for circle in circles:
        cv.circle(image, (magnitude * circle.x,  (h - magnitude *circle.y)), circle.radius * magnitude, (0, 0, 255), 2)
        cv.putText(image, index, (magnitude *circle.x, (h - magnitude *circle.y)), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
        
    for edge in edges:
        cv.circle(image, (magnitude * edge.p1x,  (h - magnitude *edge.p1y)), 5, (0, 255, 255), cv.FILLED)
        p2x = int(float(p2x))
        p2y = int(float(p2y))
        cv.circle(image, (magnitude * edge.p2x,  (h - magnitude *edge.p2y)), 5, (0, 255, 255), cv.FILLED)
        cv.line(image, (magnitude *edge.p1x, (h - magnitude *edge.p1y)), (magnitude *edge.p2x,(h - magnitude *edge.p2y)), (255, 255, 0), 3)


    cv.imshow("output", image)
    cv.waitKey(1)