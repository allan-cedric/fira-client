#include "header.h"
#include "math_operations.h"
#include <math.h>

// default positions: blue robots (left side)

// field.height/2 == 65
#define GK_DEFAULT_X 5.0
#define GK_DEFAULT_Y 65.0

#define GOAL_MAX_Y 85 //65 + 20
#define GOAL_MIN_Y 45 //65 - 20

#define GOAL_MAX_X 15.0 
#define GOAL_MIN_X -10.0


// typedef struct {
//     bool
//         especial_case, 
//         wrc, // we are closer
//         tra, // they are atacking
//         wra; // we are atacking
// } field_status_t;

// typedef struct {
//     int our_bots_n, their_bots_n;
//     ball_t ball;
//     bot_t our_bots[NUM_BOTS];
//     bot_t their_bots[NUM_BOTS];
//     bool my_robots_are_yellow;
//     field_status_t fs;
//     bot_t *closer_bot;
// } field_t;


objective_t  goalkeeper_default_position(bool is_yellow)
{
    objective_t obj;

    if (is_yellow) // our field == right
    {
        obj = {.x = 150 - GK_DEFAULT_X, .y = GK_DEFAULT_Y, .angle = 0};
    }
    else // our field == left
    {
        obj = {.x = GK_DEFAULT_X, .y = GK_DEFAULT_Y, .angle = 2* M_PI};
    }
    return obj;
}

objective_t between_goal_and_ball(ball_t ball, bool is_yellow)
{
    ball.vx
    ball.vy
    return;
}


objective_t goalkeeper_objective(field_t* field)
{
    if(!field->fs.tra) // they are not attacking, no need to worry.
        return goalkeeper_default_position(field->my_robots_are_yellow);       

    else // WE'RE UNDER ATTACK!
    {
        // we need to position the goalkeeper between the goal and the ball,
        // but it cannot go beyond the boundaries of the goal area.
        return between_goal_and_ball(field->ball, field->my_robots_are_yellow);

    }


}