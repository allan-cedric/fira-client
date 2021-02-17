#include <math.h>
#include "math_operations.cpp" // include em .cpp KKKKK

typedef struct {
    double x, y;
}  robot_t;


int main()
{
    int robots_n = 5; // all robots - 1

    circle_t circles[robots_n];
    robot_t robots[robots_n]; 

    // Generating array of circles
    for(int i = 0; i < robots_n; i++) {
        circle_t circle;
        robot_t robot = robots[i]; // from a list of robots

        circle.center.x = robot.x;
        circle.center.y = robot.y;
        circle.radius = RADIUS;

        circles[i] = circle;
    }    

    int circles_n = robots_n;

    // Getting bitangents
    for (int i = 0; i < circles_n; i++) 
        for (int j = 0; j < i; j++) {
            var internal = new InternalBitangents(circles[i], circles[j]);
            add_edge(i, internal.C, j, internal.F);
            if (circles[i].r != 0 && circles[j].r != 0) { add_edge(i, internal.D, j, internal.E); }
            var external = new ExternalBitangents(circles[i], circles[j]);
            if (circles[i].r != 0 || circles[j].r != 0) { add_edge(i, external.C, j, external.F); }
            if (circles[i].r != 0 && circles[j].r != 0) { add_edge(i, external.D, j, external.E); }
}