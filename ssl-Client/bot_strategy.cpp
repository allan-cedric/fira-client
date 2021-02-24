
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

float_pair our_goal_pair(bool mray)
{
    return {.x = our_goal_x(mray), .y = our_goal_y()};
}

float_pair their_goal_pair(bool mray)
{
    return {.x = their_goal_x(mray), .y = their_goal_y()};
}

void send_bot_to(bot_t *b, objective_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = dest.angle;
}

objective_t get_interception_point(line_t ball_l, float_pair ball_p, double dist, bool mray)
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

line_t get_line(float_pair a, float_pair b)
{
    double res_a = (a.y - b.y) 
            / (a.x - b.x);
    double res_b = b.y - res_a * b.x;
    return {.a = res_a, .b = res_b};
}

line_t get_line_from_vec(float_pair p, float_pair v){

    float_pair u = vec_add(p,v);
    return get_line(p,u);

}

void set_bot_strategies(field_t *f)
{
    bool mray = f->my_robots_are_yellow;

    // set both non goalkeeper bots that will do something
    bot_t *dominant = f->closer_bot;
    bot_t *auxiliary = f->closer_bot == &f->our_bots[1] 
                        ? &f->our_bots[2] : &f->our_bots[1];

    // ball pair
    float_pair ball_p = {.x = f->ball.x, .y = f->ball.y};

    // ========= TODO goalkeeper do his stuff ==========
    if (dominant == &f->our_bots[0]) {
        // kick_away_from_everyone();
    }
    // ========= TODO goalkeeper do his stuff ==========
    
    // line Y = aX + b 
    // pass through ball and their goal
    line_t atk_line = get_line(ball_p, their_goal_pair(mray));

    // pass through ball and our goal
    line_t def_line = get_line(ball_p, our_goal_pair(mray));

    objective_t ball_atk_o = get_interception_point(
                            atk_line, ball_p, ATK_DISP_DIST, mray);

    objective_t ball_def_p = get_interception_point(
                            def_line, ball_p, DEF_DISP_DIST, mray);

    // if (f->fs.wrc) {
    //     // go_to_goal(dominant);
    // }

    // if (f->fs.wra) {
    //     // go_to_goal(dominant);
    // }

    // if (f->fs.tra) {
    //     // if (dominant is close enough to intercept)
    //         // intercepr_to_defend(dominant, ball_def_p);
    //     // else 
    //         // intercept_to_defend(auxiliary, ball_def_p);
    // }

    // if (f->our_bots[0].x > ball_p.x ) {
    //     f->our_bots[0].wants_to_hit_ball = true;
    // }

    if (vec_distance(ball_p, {.x = f->our_bots[0].x, .y = f->our_bots[0].y}) > 15.0){
        send_bot_to(&f->our_bots[0], ball_atk_o);
    } else {
        // if (vec_distance(ball_p, {.x = f->our_bots[0].x, .y = f->our_bots[0].y}) > 8){
        send_bot_to(&f->our_bots[0], {.x = ball_p.x, .y = ball_p.y, .angle = 0});
        // } else {
        //     send_bot_to(&f->our_bots[0], {.x = their_goal_x(mray), .y = their_goal_y(), .angle = 0});
        // }
    }

}