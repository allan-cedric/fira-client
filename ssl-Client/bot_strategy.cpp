
#include "header.h"
#include "bot_strategy.h"
#include "math_operations.h"
#include "analyzer.h"

void send_bot_to(bot_t *b, objective_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = dest.angle;
}

double our_goal_x(bool mray)
{
    return mray ? YELLOW_GOAL_X : BLUE_GOAL_X;
}

double their_goal_x(bool mray)
{
    return !mray ? YELLOW_GOAL_X : BLUE_GOAL_X;
}

void set_bot_strategies(field_t *f)
{

    // goalkeeper do his stuff

    bot_t *dominant = f->closer_bot;
    bot_t *auxiliary = f->closer_bot == &f->our_bots[1] 
                        ? &f->our_bots[2] : &f->our_bots[1];

    objective_t ball_p = {.x = f->ball.x, .y = f->ball.y, .angle = 0};

    bool mray = f->my_robots_are_yellow;

    line_t atk_line;
    atk_line.a = (ball_p.y - GOAL_Y) / (ball_p.x - their_goal_x(mray));
    atk_line.b = GOAL_Y - atk_line.a * their_goal_x(mray);

    line_t def_line;
    def_line.a = (ball_p.y - GOAL_Y) / (ball_p.x - our_goal_x(mray));
    def_line.b = GOAL_Y - def_line.a * our_goal_x(mray);

    if (dominant == &f->our_bots[0]) {
        // kick_away_from_everyone();
    }

    if (f->fs.wrc) {

    }

    if (f->fs.wra) {
        // atack();
    }

    if (f->fs.tra) {
        // intercepr_to_defend();
    }

}