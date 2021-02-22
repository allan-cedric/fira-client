
#include "header.h"
#include "bot_strategy.h"
#include "math_operations.h"

void send_bot_to(bot_t *b, float_pair dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
}

void set_bot_strategies(field_t *f)
{

    // goalkeeper do his stuff

    if (f->fs.wrc) {
        // find direction to go and align
        float_pair ball_p = {.x = f->ball.x, .y = f->ball.y};
        send_bot_to(f->closer_bot, ball_p);
    }

    if (f->fs.wra) {
        // our closer go to ball
        float_pair ball_p = {.x = f->ball.x, .y = f->ball.y};
        send_bot_to(f->closer_bot, ball_p);

        // second closer go to atack
        float_pair atack_pos = {.x = ATACK_X, .y = ATACK_Y};
        send_bot_to(f->closer_bot, atack_pos);
    }

    if (f->fs.tra) {
        // our closer go to ball
        float_pair ball_p = {.x = f->ball.x, .y = f->ball.y};
        send_bot_to(f->closer_bot, ball_p);

        // second closer go to defend
        float_pair defense_pos = {.x = DEFENSE_X, .y = DEFENSE_Y};
        send_bot_to(f->closer_bot, defense_pos);
    }

    // else:

}