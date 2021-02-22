// Made by Artur Coelho for Yapira UFPR on Feb 2021 
// for the FIRASim simulator competition 
// on the virtual IRONCUP 2021

#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"
#include "math_operations.h"

inline double get_len(double base)
{
    return (length + base) * 100;
}

inline double get_wid(double base)
{
    return (width + base) * 100;
}

void print_bot_info(fira_message::sim_to_ref::Robot bots[NUM_BOTS], int color)
{
    for (int i = 0; i < NUM_BOTS; i++){
        printf("%s: %d ", color ? "Blue" : "Yellow", i);
        printf("x: %f y: %f a: %f \n", get_len(bots[i].x()), get_wid(bots[i].y()), bots[i].orientation());
        printf("vx: %f vy: %f va: %f \n", bots[i].vx(), bots[i].vy(), bots[i].vorientation());
    }
    printf("\n");
}

void print_ball_info(fira_message::sim_to_ref::Ball ball)
{
    printf("Ball:\n");
    printf("x: %f y: %f\n", get_len(ball.x()), get_wid(ball.y()));
    printf("vx: %f vy: %f\n", ball.vx(), ball.vy());
    printf("\n");
}


int max_dist_index(double d[NUM_BOTS])
{
    int max = 0;
    for (int i = 0; i < NUM_BOTS; i++)
        if (d[max] < d[i])
            max = i;
    return max;
}

int min_dist_index(double d[NUM_BOTS])
{
    int min = 0;
    for (int i = 0; i < NUM_BOTS; i++)
        if (d[min] > d[i])
            min = i;
    return min;
}

int we_are_closer(fira_message::sim_to_ref::Robot our_bots[NUM_BOTS], 
                fira_message::sim_to_ref::Robot their_bots[NUM_BOTS], 
                fira_message::sim_to_ref::Ball ball )
{
    double our_distances[NUM_BOTS];
    double their_distances[NUM_BOTS];
    float_pair ball_p = {.x = ball.x(), .y = ball.y() };

    for (int i = 0; i < NUM_BOTS; i++) {
        float_pair bot_p = {.x = our_bots[i].x(), .y = our_bots[i].y() };
        our_distances[i] = vec_distance(bot_p, ball_p);
    }

    for (int i = 0; i < NUM_BOTS; i++) {
        float_pair bot_p = {.x = their_bots[i].x(), .y = their_bots[i].y() };
        their_distances[i] = vec_distance(bot_p, ball_p);
    }

    return our_distances[min_dist_index(our_distances)] < their_distances[min_dist_index(their_distances)];
}   

int field_analyzer(fira_message::sim_to_ref::Frame detection, bool mray )
{
    fira_message::sim_to_ref::Ball ball = detection.ball();
    fira_message::sim_to_ref::Robot blue_bots[NUM_BOTS];
    fira_message::sim_to_ref::Robot yellow_bots[NUM_BOTS];

    for (int i = 0; i < NUM_BOTS; i++) {
        blue_bots[i] = detection.robots_blue(i);
        yellow_bots[i] = detection.robots_yellow(i);
    }

    int wrc = we_are_closer(mray ? yellow_bots : blue_bots, 
                    mray ? blue_bots : yellow_bots, 
                    ball);

    printf("%s WRC: %d\n",mray ? "y" : "b",  wrc);
    print_ball_info(ball);
    print_bot_info(blue_bots, 1);
    print_bot_info(yellow_bots, 0);

    return 1;
}