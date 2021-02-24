#ifndef STRATEGY
#define STRATEGY

#include "header.h"
#include "math_operations.h"

#define YELLOW_GOAL_X 160.0
#define BLUE_GOAL_X 10.0
#define GOAL_Y 65.0

#define ATK_DISP_DIST 6.0
#define DEF_DISP_DIST 15.0

#define GOAL_DIFF_CORRECTION 0.225

#define HEIGHT_ACEPTANCE 10

typedef struct {
    double a, b;
} line_t;

void set_bot_strategies(field_t*);

double our_goal_x(bool mray);

double their_goal_x(bool mray);

double their_goal_y();

double our_goal_y();

float_pair_t our_goal_pair(bool mray);

float_pair_t their_goal_pair(bool mray);

void send_bot_to(bot_t *b, objective_t dest);

objective_t get_interception_point(line_t ball_l, float_pair_t ball_p, double dist, bool mray);

line_t get_line(float_pair_t a, float_pair_t b);

line_t get_line_from_vec(float_pair_t p, float_pair_t v);

void set_bot_strategies(field_t *f);

#endif