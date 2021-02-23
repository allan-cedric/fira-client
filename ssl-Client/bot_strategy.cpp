
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
    line_t atk_line;
    atk_line.a = (ball_p.y - their_goal_y()) 
                / (ball_p.x - their_goal_x(mray));
    atk_line.b = their_goal_y() - atk_line.a * their_goal_x(mray);

    // pass through ball and our goal
    line_t def_line;
    def_line.a = (ball_p.y - our_goal_y()) 
                / (ball_p.x - our_goal_x(mray));
    def_line.b = our_goal_y() - def_line.a * our_goal_x(mray);

    objective_t ball_atk_p = get_interception_point(
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

    f->our_bots[0].obj = ball_atk_p;

}