
#include "header.h"
#include "bot_strategy.h"
#include "math_operations.h"
#include "analyzer.h"
#include "goalkeeper.h"

// set each goal mid positions based on side
double our_goal_x(bool mray)
{
    return mray ? YELLOW_GOAL_X : BLUE_GOAL_X;
}

double their_goal_x(bool mray)
{
    return !mray ? YELLOW_GOAL_X : BLUE_GOAL_X;
}

// TODO check a better y
double their_goal_y()
{
    return GOAL_Y;
}

// TODO check a better y
double our_goal_y()
{
    return GOAL_Y;
}

float_pair_t our_goal_pair(bool mray)
{
    return {.x = our_goal_x(mray), .y = our_goal_y()};
}

float_pair_t their_goal_pair(bool mray)
{
    return {.x = their_goal_x(mray), .y = their_goal_y()};
}

float_pair_t their_goal_pair_with_correction(bool mray, float_pair_t ball_p)
{
    double x = their_goal_x(mray);
    double y = their_goal_y() + (their_goal_y() - ball_p.y) * GOAL_DIFF_CORRECTION;
    return {.x = x, .y = y};
}

// check if bot is above or below the ball
// checks accordinly to side and allows for error
bool is_aligned_to_goal(float_pair_t ball_p, float_pair_t bot_p)
{
    if (ball_p.y > their_goal_y()) {
        return bot_p.y > ball_p.y - HEIGHT_ACEPTANCE;
    } 
    return bot_p.y < ball_p.y + HEIGHT_ACEPTANCE;
}

// set a bot objective
void send_bot_to(bot_t *b, objective_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = dest.angle;
}

// get a point on the same line, behind the ball
objective_t get_interception_point(line_t ball_l, 
                                    float_pair_t ball_p, 
                                    double dist, 
                                    bool mray )
{
    double dist_x = sqrt(dist*dist / (1 + ball_l.a*ball_l.a));
    double target_x = !mray ? ball_p.x - dist_x : ball_p.x + dist_x;

    double target_y = ball_l.a * target_x + ball_l.b;

    double incl = atan(ball_l.a);

    return {.x = target_x,
             .y = target_y,
             .angle = incl };
            //  .angle = M_PI_4 };
}

// gets a line reduced equation based on two points
line_t get_line(float_pair_t a, float_pair_t b)
{
    double res_a = (a.y - b.y) 
            / (a.x - b.x);
    double res_b = b.y - res_a * b.x;
    return {.a = res_a, .b = res_b};
}

// get a reduced line equation based on point and 
// director vector
line_t get_line_from_vec(float_pair_t p, float_pair_t v)
{
    float_pair_t u = vec_add(p,v);
    return get_line(p,u);
}

// calculates a distance for the bot to go behind the ball
double get_atack_diff(line_t ball_l, line_t bot_l)
{
    double ball_incl = atan(ball_l.a);
    double bot_incl = atan(bot_l.a);
    return ATK_DISP_DIST * (1 + fabs(ball_incl) + fabs(bot_incl));
}

void set_bot_strategies(field_t *f)
{
    bool mray = f->my_robots_are_yellow;
    
    // set each bot pointer
    bot_t *goalkeeper, *dominant, *auxiliary, *aux2;
    
    goalkeeper = &f->our_bots[0]; // ALWAYS
    dominant = f->closer_bot; // Defined on we_are_closer (analyzer.cpp)

    if (goalkeeper == dominant) {
        auxiliary = &f->our_bots[1];
        aux2 = &f->our_bots[2];
    } else {
        if (dominant == &f->our_bots[1]) {
            auxiliary = &f->our_bots[2];
        } else {
            auxiliary = &f->our_bots[1];
        }
        aux2 = auxiliary;
    }

    // set pairs
    float_pair_t ball_p = {.x = f->ball.x, .y = f->ball.y};
    float_pair_t ball_vec = {.x = f->ball.vx, .y = f->ball.vy};
    float_pair_t dom_bot_p = {.x = dominant->x, .y = dominant->y};
    float_pair_t aux_bot_p = {.x = auxiliary->x, .y = auxiliary->y};
    float_pair_t aux2_bot_p = {.x = aux2->x, .y = aux2->y};

    // line Y = aX + b 
    // pass through ball and goal
    line_t atk_line = get_line(ball_p, their_goal_pair_with_correction(mray, ball_p));
    line_t def_line = get_line(ball_p, our_goal_pair(mray));
    line_t bot_to_ball_line = get_line(ball_p, dom_bot_p);
    line_t ball_line = get_line_from_vec(ball_p, ball_vec);

    double atk_diff = get_atack_diff(atk_line, bot_to_ball_line);
    objective_t ball_atk_o = get_interception_point(
                            atk_line, ball_p, atk_diff, mray);

    // TODO
    objective_t ball_def_o = get_interception_point(
                            def_line, ball_p, DEF_DISP_DIST, mray);

    // ============= SEND BOT TO OBJ ================ //

    // goalkeeper standart procedure
    send_bot_to(goalkeeper, goalkeeper_objective(f));

    // dominant and aux procedures
    if (vec_distance(ball_p, {.x = dominant->x, .y = dominant->y}) 
                    > atk_diff * 1.5 || !is_aligned_to_goal(ball_p, dom_bot_p)) {
        send_bot_to(dominant, ball_atk_o);
    } else {
        send_bot_to(dominant, {.x = ball_p.x, .y = ball_p.y, .angle = 0});
    }

}