#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

const double width = 1.3 / 2.0, length = 1.7 / 2.0;

inline double get_len(double base)
{
    return (length + base) * 100;
}

inline double get_wid(double base)
{
    return (width + base) * 100;
}

void print_bot_info(fira_message::sim_to_ref::Robot bots[3], int color)
{

    for (int i = 0; i < 3; i++){
        printf("%s: %d ", color ? "Blue" : "Yellow", i);
        printf("x: %f y: %f a: %f \n", get_len(bots[i].x()), get_wid(bots[i].y()), bots[i].orientation());
        printf("vx: %f vy: %f va: %f \n", bots[i].vx(), bots[i].vy(), bots[i].vorientation());
    }
    printf("\n");
}

int field_analyzer(fira_message::sim_to_ref::Frame detection)
{
    fira_message::sim_to_ref::Ball ball = detection.ball();
    fira_message::sim_to_ref::Robot blue_bots[3];
    fira_message::sim_to_ref::Robot yellow_bots[3];
    for (int i = 0; i < 3; i++) {
        blue_bots[i] = detection.robots_blue(i);
        yellow_bots[i] = detection.robots_yellow(i);
    }

    print_bot_info(blue_bots, 1);
    print_bot_info(yellow_bots, 0);

    return 1;
}