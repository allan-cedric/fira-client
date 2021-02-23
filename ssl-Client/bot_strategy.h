#ifndef STRATEGY
#define STRATEGY

#include "header.h"

#define YELLOW_GOAL_X 160.0
#define BLUE_GOAL_X 10.0
#define GOAL_Y 65.0

typedef struct {
    double a, b;
} line_t;

void set_bot_strategies(field_t*);

#endif