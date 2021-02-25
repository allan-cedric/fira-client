
#include "header.h"
#include "bot_strategy.h"
#include "math_operations.h"
#include "analyzer.h"
#include "goalkeeper.h"
#include "bot_execute.h"

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

double aux_atk_dist_x(bool mray)
{
    return mray
            ? AUX_DIST_X 
            : -AUX_DIST_X;
}       

double aux_atk_dist_y(float_pair_t p)
{
    return p.y > their_goal_y() 
        ? -AUX_DIST_Y
        : AUX_DIST_Y;
}

double aux_def_dist_x(bool mray)
{
    return mray
            ? AUX_DIST_X 
            : -AUX_DIST_X;
}

double aux_def_dist_y(float_pair_t p)
{
    return p.y > our_goal_y() 
        ? -AUX_DIST_Y
        : AUX_DIST_Y;
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

// check if bot is behind ball
bool is_behind_ball(float_pair_t ball_p, float_pair_t bot_p, bool mray)
{
    return mray 
        ? ball_p.x + 1 < bot_p.x // yellow
        : ball_p.x - 1 > bot_p.x; // blue
}

bool is_in_goal_area(float_pair_t p, bool mray)
{
    if (mray) {
        return p.x > 150 - GOAL_MAX_X * 1.1;
    } else {
        return p.x < GOAL_MAX_X * 1.1;
    }
}

// set a bot objective
void send_bot_to(bot_t *b, objective_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = dest.angle;
}

// set a bot objective with pair
void send_bot_to(bot_t *b, float_pair_t dest)
{
    b->obj.x = dest.x;
    b->obj.y = dest.y;
    b->obj.angle = 0;
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

// ponto mais proximo do ponto na reta
float_pair_t point_on_line(line_t line, float_pair_t point)
{
    // y = ax + b
    // double dist = fabs(-line.a * point.x + point.y - line.b) 
    //         / sqrt(line.a*line.a + line.b*line.b);

    double target_x = (line.b - point.y + (-1 / line.a) * point.x)
                        / ((-1 / line.a) - line.a);
    double target_y = line.a * target_x + line.b;
    return {.x = target_x, .y = target_y};
}

void fix_obj_to_keep_outside_goal_area(objective_t *obj, bool mray)
{
    if (mray) {
        if (obj->x > (150 - GOAL_MAX_X ) - AREA_ERROR) {
            obj->x = (150 - GOAL_MAX_X ) - AREA_ERROR;
            // if (obj->y > GOAL_MAX_Y + AREA_ERROR) obj->y = GOAL_MAX_Y + AREA_ERROR;
            // if (obj->y < GOAL_MIN_Y - AREA_ERROR) obj->y = GOAL_MIN_Y - AREA_ERROR;
        }
    } else {
        if (obj->x < GOAL_MAX_X + AREA_ERROR){
            obj->x = GOAL_MAX_X + AREA_ERROR;
            // if (obj->y > GOAL_MAX_Y + AREA_ERROR) obj->y = GOAL_MAX_Y + AREA_ERROR;
            // if (obj->y < GOAL_MIN_Y - AREA_ERROR) obj->y = GOAL_MIN_Y - AREA_ERROR;
        }
    }

}

void set_bot_strategies(field_t *f)
{
    bool mray = f->my_robots_are_yellow;
    
    // set each bot pointer
    bot_t *goalkeeper, *dominant, *auxiliary, *aux2;

    goalkeeper = &f->our_bots[0]; // ALWAYS
    dominant = f->closer_bot; // Defined on we_are_closer (analyzer.cpp)

    if (goalkeeper == dominant) { // goalkeeper has the ball
        auxiliary = &f->our_bots[1]; // others do not interfere
        aux2 = &f->our_bots[2];
    } else {
        if (dominant == &f->our_bots[1]) { // set other auxiliary
            auxiliary = &f->our_bots[2];
        } else {
            auxiliary = &f->our_bots[1];
        }
        aux2 = auxiliary;
    }

    // set pairs
    float_pair_t ball_p = {.x = f->ball.x, .y = f->ball.y};
    float_pair_t ball_vec = {.x = f->ball.vx, .y = f->ball.vy};
    objective_t ball_obj = {.x = f->ball.x, .y = f->ball.y, .angle = 0};

    float_pair_t dom_bot_p = {.x = dominant->x, .y = dominant->y};
    float_pair_t aux_bot_p = {.x = auxiliary->x, .y = auxiliary->y};
    float_pair_t aux2_bot_p = {.x = aux2->x, .y = aux2->y};

    // line Y = aX + b 
    // pass through ball and goal
    line_t atk_line = get_line(ball_p, their_goal_pair_with_correction(mray, ball_p));
    line_t def_line = get_line(ball_p, our_goal_pair(mray));
    // dom bot to ball
    line_t bot_to_ball_line = get_line(ball_p, dom_bot_p);
    // bal trajectory
    line_t ball_line = get_line_from_vec(ball_p, ball_vec);

    // a point behind the ball to their goal
    double atk_diff = get_atack_diff(atk_line, bot_to_ball_line);
    objective_t ball_atk_o = get_interception_point(
                            atk_line, ball_p, atk_diff, mray);

    // a point in front of the ball away from our goal
    objective_t ball_def_o = get_interception_point(
                            def_line, ball_p, DEF_DISP_DIST, mray);

    objective_t ball_def_o2 = {.x = ball_def_o.x, 
                                .y = ball_def_o.y 
                                + (ball_def_o.y > our_goal_y() ? 10 : -10 )};
                                // temp hack fix

    // ============= SEND BOT TO OBJ ================ //

    if (!is_on_my_field(ball_p.x, mray)){

        // attack procedure

        if (vec_distance(ball_p, dom_bot_p) > atk_diff * 1.5 
            || !is_aligned_to_goal(ball_p, dom_bot_p)
            /*|| vec_distance(ball_p, dom_bot_p) < atk_diff * 1.1*/) {
            fix_obj_to_keep_outside_goal_area(&ball_atk_o, mray);
            send_bot_to(dominant, ball_atk_o);
        } else {
            fix_obj_to_keep_outside_goal_area(&ball_obj, mray);
            send_bot_to(dominant, ball_obj);
        }

        objective_t aux_obj = {.x = ball_p.x + aux_atk_dist_x(mray),
                                .y = ball_p.y + aux_atk_dist_y(dom_bot_p)};

        if (is_in_goal_area({aux_obj.x, aux_obj.y}, !mray)) {
            if (mray) {
                aux_obj = { .x = DEF_POS_X, .y = DEF_POS_Y };
            } else {
                aux_obj = { .x = ATK_POS_X, .y = ATK_POS_Y };
            }
        }

        if (vec_distance(ball_p, their_goal_pair(mray)) < 20){
            aux_obj = {ball_p.x, ball_p.y};
        }

        if (fabs(ball_line.a) > 3.0) {
            float_pair_t p = point_on_line(ball_line, aux_bot_p);
            aux_obj = {.x = p.x, .y = p.y};
        }
        
        send_bot_to(auxiliary, aux_obj);

    } else {

        // defense procedures
        if (vec_distance(ball_p, dom_bot_p) > DEF_DISP_DIST) {
            fix_obj_to_keep_outside_goal_area(&ball_def_o2, mray);
            send_bot_to(dominant, ball_def_o2);
            dominant->wants_to_hit_ball = false;
        } else {
            fix_obj_to_keep_outside_goal_area(&ball_obj, mray);
            send_bot_to(dominant, ball_obj);
            if (is_behind_ball(ball_p, dom_bot_p, mray)) {
                dominant->wants_to_hit_ball = true;
            }
        }

        objective_t aux_obj = {.x = ball_p.x + aux_def_dist_x(mray),
                        .y = ball_p.y + aux_def_dist_y(dom_bot_p)};


        if (is_in_goal_area({aux_obj.x, aux_obj.y}, mray)) {
            if (mray) {
                aux_obj = { .x = ATK_POS_X, .y = ATK_POS_Y };
            } else {
                aux_obj = { .x = DEF_POS_X, .y = DEF_POS_Y };
            }
        }

        send_bot_to(auxiliary, aux_obj);

    }

    if (auxiliary != aux2) {
        objective_t aux2_obj;
        if (mray) {
            aux2_obj = { .x = ATK_POS_X, .y = ATK_POS_ALT_Y };
        } else {
            aux2_obj = { .x = DEF_POS_X, .y = DEF_POS_ALT_Y };
        }
        send_bot_to(aux2, aux2_obj);
    }
    
    // goalkeeper standart procedure 
    // overwrites previous dominant behaviour
    send_bot_to(goalkeeper, goalkeeper_objective(f));

    // printf("d %d %f\n", dominant->index, dominant->x);
    // printf("g %d\n", goalkeeper->index);
    // printf("a %d\n", auxiliary->index);
    // printf("2 %d\n", aux2->index);
    // printf("\n");

}