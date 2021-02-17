#include "net/robocup_ssl_client.h"
#include <stdio.h>

#define OCCUPIED 'X'
#define FREE '-'

char grid[150][130];

void free_grid(char grid[][130])
{
    int i,j;
    for (i = 0; i <= 149; i++)
        for(j = 0; j <= 129; j++)
        {
            grid[i][j] = FREE;
        }
}


void map_robot(char grid[][130], const fira_message::sim_to_ref::Robot &robot)
{
    float raw_x = robot.x();
    float raw_y = robot.y();

    // printf("%f, %f\n", raw_x, raw_y);

    int grid_x = int(raw_x);
    int grid_y = int(raw_y);

    // printf("%d, %d\n", grid_x, grid_y);


    int i,j;
    for(i = grid_x -5; i <= grid_x + 5; i++)
        for(j = grid_y -5; j <= grid_y + 5; j++)
        {
            if ((0 <= i && i <= 149) && (0 <= j && j <= 129))
                grid[i][j] = OCCUPIED;
        }

}

void print_grid(char grid[][130])
{
    int i,j;
    for (i = 0; i <= 149; i++)
    {
        for(j = 0; j <= 129; j++)
        {
            printf("%c ", grid[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}