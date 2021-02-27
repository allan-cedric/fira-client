#include "header.h"
#include "math_operations.h"
#include <math.h>
#include "bot_strategy.h"

#include "goalkeeper.h"

objective_t  goalkeeper_default_position(bool is_yellow)
{
    objective_t obj;m algum campo e por favor sempre peçam para os juizes abrirem e conferirem com vocês pra evitar problemas.

    if (is_yellow) // our field == right
    {
        obj = {.x = 160 - GK_DEFAULT_X + 15.0, .y = GK_DEFAULT_Y, .angle = 2* M_PI};
    }
    else // our field == left
    {
        obj = {.x = GK_DEFAULT_X, .y = GK_DEFAULT_Y, .angle = 0};
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
        goal_area_min_x = FIELD_LENGTH - GOAL_MIN_X;
        goal_area_max_x = FIELD_LENGTH - GOAL_MAX_X;


        if (ball_vec.x > 0) // if the ball is going to the goal
        {

            // if ball is in the goal area
            if( inrange(goal_area_max_x, goal_area_min_x, ball.x) )
            {
                // prediction to when it is going to hit the boundary
                ball_predicted_x = 160; // lets predict its 'y' coordinate for the goal
                ball_predicted_y = ball_path.a * 160 + ball_path.b;
            }
            else // ball still outside goal area
            {
                ball_predicted_x  = goal_area_max_x; // lets predict its 'y' coordinate for the goal area
                ball_predicted_y = ball_path.a * goal_area_max_x + ball_path.b;
            }
        
            // check whether these coordinates are in the goal area
            // (otherwise clamp them)
        }
        else // ball not going to the goal
        {
            ball_predicted_x = goalkeeper.x;
            ball_predicted_y = ball.y; // just kinda follow it
        }

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

        if (ball_vec.x < 0) // if the ball is going to the goal
        {

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
        }
        else // ball not going to the goal
        {
            ball_predicted_x = goalkeeper.x;
            ball_predicted_y = ball.y; // just kinda follow it
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

bool gk_should_follow_ball(bool is_yellow, bot_t* gk, ball_t ball)
// return true if the ball is closer than GK_BALL_DIST
// and the ball is in its front
// and goalkeeper still in GK_GOAL_DIST range
{
    float_pair_t gk_coord = {.x = gk->x, .y = gk->y};
    float_pair_t ball_coord = {.x = ball.x, .y = ball.y};

    if(vec_distance(gk_coord, ball_coord) < GK_BALL_DIST)
    // if it is close enough
    {
        if (is_yellow)
        {
            if ( (ball.x < gk->x) && (FIELD_LENGTH - GK_GOAL_DIST) < gk->x)
                return true;
        }
        else
        {
            if ( (gk->x < ball.x) && gk->x < GK_GOAL_DIST )
                return true;
        }
    }

    return false;
}


objective_t goalkeeper_objective(field_t* field)
{
    bot_t* goalkeeper = &(field->our_bots[0]); 

    objective_t obj;
    if(!field->fs.tra) {// they are not attacking, no need to worry.
        obj =  goalkeeper_default_position(field->my_robots_are_yellow);       
    }
    else
    {
        // WE'RE UNDER ATTACK!
        // we need to position the goalkeeper between the goal and the ball,
        // but it cannot go beyond the boundaries of the goal area.

        if (gk_should_follow_ball(field->my_robots_are_yellow, goalkeeper, field->ball))
        {
            // follow the ball (kick it)
            obj.x = field->ball.x;
            obj.y = field->ball.y;
            goalkeeper->wants_to_hit_ball = true;         
        }
        else
        {
            obj =  between_goal_and_ball(*goalkeeper, field->ball,
                                    field->my_robots_are_yellow, &(goalkeeper->wants_to_hit_ball));
        }
    }
    // printf("GK objective: %f,  %f\n", obj.x, obj.angle);

    return obj;

}