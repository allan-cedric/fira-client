
#include "header.h"
#include "bot_strategy.h"
#include "math_operations.h"
#include "analyzer.h"

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

bool ball_is_aligned_to_goal(float_pair_t ball_p, float_pair_t bot_p)
{
    if (ball_p.y > their_goal_y()) {
        return bot_p.y > ball_p.y - HEIGHT_ACEPTANCE;
    } 
    return bot_p.y < ball_p.y + HEIGHT_ACEPTANCE;
}

void send_bot_to(bot_t *b, objective_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = dest.angle;
}

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

line_t get_line(float_pair_t a, float_pair_t b)
{
    double res_a = (a.y - b.y) 
            / (a.x - b.x);
    double res_b = b.y - res_a * b.x;
    return {.a = res_a, .b = res_b};
}

line_t get_line_from_vec(float_pair_t p, float_pair_t v)
{
    float_pair_t u = vec_add(p,v);
    return get_line(p,u);
}

double get_atack_diff(line_t ball_l, line_t bot_l)
{
    double ball_incl = atan(ball_l.a);
    double bot_incl = atan(bot_l.a);
    return ATK_DISP_DIST * (1 + fabs(ball_incl) + fabs(bot_incl));
}

void set_bot_strategies(field_t *f)
{
    bool mray = f->my_robots_are_yellow;

    // set both non goalkeeper bots that will do something
    bot_t *dominant = f->closer_bot;
    bot_t *auxiliary = f->closer_bot == &f->our_bots[1] 
                        ? &f->our_bots[2] : &f->our_bots[1];

    // ball pair
    float_pair_t ball_p = {.x = f->ball.x, .y = f->ball.y};
    float_pair_t bot_p = {.x = f->our_bots[0].x, .y = f->our_bots[0].y};

    // line Y = aX + b 
    // pass through ball and goal
    line_t atk_line = get_line(ball_p, their_goal_pair_with_correction(mray, ball_p));
    line_t def_line = get_line(ball_p, our_goal_pair(mray));
    line_t bot_to_ball_line = get_line(ball_p, bot_p);

    // ========= TODO goalkeeper do his stuff ==========
    if (dominant == &f->our_bots[0]) {
        // kick_away_from_everyone();
    }
    // ========= TODO goalkeeper do his stuff ==========


    double atk_diff = get_atack_diff(atk_line, bot_to_ball_line);
    objective_t ball_atk_o = get_interception_point(
                            atk_line, ball_p, atk_diff, mray);

    objective_t ball_def_p = get_interception_point(
                            def_line, ball_p, DEF_DISP_DIST, mray);

    if (vec_distance(ball_p, {.x = f->our_bots[0].x, .y = f->our_bots[0].y}) 
                    > atk_diff * 1.5 || !ball_is_aligned_to_goal(ball_p, bot_p)) {
        send_bot_to(&f->our_bots[0], ball_atk_o);
    } else {
        send_bot_to(&f->our_bots[0], {.x = ball_p.x, .y = ball_p.y, .angle = 0});
    }

}