#include "header.h"
#include "math_operations.h"
#include <math.h>
#include "bot_strategy.h"

#include "goalkeeper.h"

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

objective_t between_goal_and_ball(bot_t goalkeeper, ball_t ball, bool is_yellow, bool* should_hit_ball)
{
    objective_t obj;

    float_pair_t ball_center = {.x = ball.x, .y = ball.y };
    float_pair_t ball_vec = {.x = ball.vx, .y = ball.vy};
    line_t ball_path = get_line_from_vec(ball_center, ball_vec);

    // once we have the ball path, we can try to predict the ball's location,
    // if the ball is outside the goal area, the robot is to wait for the ball
    //  on the boundaries of the area.
    // otherwise, it will try to stop it before the ball hits the goal. 

    // according to our field, get the goal area:
    // (and) check whether the ball is already on the goal area:
    double goal_area_min_x;
    double goal_area_max_x;

    double ball_predicted_x;
    double ball_predicted_y;


    if (is_yellow)
    {
        goal_area_min_x = 150.0 - GOAL_MIN_X;
        goal_area_max_x = 150.0 - GOAL_MAX_X;
        // if ball is in the goal area
        if( inrange(goal_area_max_x, goal_area_min_x, ball.x) )
        {
            // prediction to when it is going to hit the boundary
            ball_predicted_x = 150; // lets predict its 'y' coordinate for the goal
            ball_predicted_y = ball_path.a * 150 + ball_path.b;
        }
        else // ball still outside goal area
        {
            ball_predicted_x  = goal_area_max_x; // lets predict its 'y' coordinate for the goal area
            ball_predicted_y = ball_path.a * goal_area_max_x + ball_path.b;
        }
    
        // check whether these coordinates are in the goal area
        // (otherwise clamp them)

        if ( ball_predicted_y < GOAL_MIN_Y )
            obj.y =  GOAL_MIN_Y;

        else if ( ball_predicted_y > GOAL_MAX_Y )
            obj.y =  GOAL_MAX_Y;

        else
            obj.y = ball_predicted_y;

        obj.x = ball_predicted_x;

        // the robot shall hit the ball if the
        // ball is closer to the goal than it is
        *should_hit_ball = ball.x > goalkeeper.x;


    }
    else
    {
        goal_area_min_x = GOAL_MIN_X;
        goal_area_max_x = GOAL_MAX_X;
        // if ball should be in the goal area
        if( inrange(goal_area_min_x, goal_area_max_x, ball.x) )
        {
            // prediction to when it is going to hit the boundary
            ball_predicted_x = 0; // lets predict its 'y' coordinate for the goal
            ball_predicted_y = ball_path.a * 0 + ball_path.b;
        }
        else // ball still outside goal area
        {
            ball_predicted_x  = goal_area_max_x; // lets predict its 'y' coordinate for the goal area
            ball_predicted_y = ball_path.a * goal_area_max_x + ball_path.b;
        }
    
        // check whether these coordinates are in the goal area
        // (otherwise clamp them)

        if ( ball_predicted_y < GOAL_MIN_Y )
            obj.y =  GOAL_MIN_Y;

        else if ( ball_predicted_y > GOAL_MAX_Y )
            obj.y =  GOAL_MAX_Y;

        else
            obj.y = ball_predicted_y;

        obj.x = ball_predicted_x;


        // the robot shall hit the ball if the
        // ball is closer to the goal than it is
        *should_hit_ball = ball.x < goalkeeper.x;
    }
    
    return obj;
}

objective_t goalkeeper_objective(field_t* field)
{
    if(!field->fs.tra) {// they are not attacking, no need to worry.
        return goalkeeper_default_position(field->my_robots_are_yellow);       

    } else { // WE'RE UNDER ATTACK!
        // we need to position the goalkeeper between the goal and the ball,
        // but it cannot go beyond the boundaries of the goal area.
        return between_goal_and_ball(field->our_bots[0], field->ball,
                                    field->my_robots_are_yellow, &field->our_bots[0].wants_to_hit_ball);

    }

}