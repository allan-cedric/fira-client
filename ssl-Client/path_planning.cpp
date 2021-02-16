#include "net/robocup_ssl_client.h"
#include <stdio.h>

int grid[150][130];

int map_robot(int **grid, const fira_message::sim_to_ref::Robot &robot)
{
    float raw_x = robot.x();
    float raw_y = robot.y();

    printf("%f, %f", raw_x, raw_y);

    int grid_x = int(raw_x) - 75;
    int grid_y = int(raw_y) - 65;



}

