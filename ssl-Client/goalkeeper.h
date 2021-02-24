#ifndef GOALKEEPER
#define GOALKEEPER

#include "header.h"
#include "bot_strategy.h"

// field.height/2 == 65
#define GK_DEFAULT_X 5.0
#define GK_DEFAULT_Y 65.0

#define GOAL_MAX_Y 85 //65 + 20
#define GOAL_MIN_Y 45 //65 - 20

#define GOAL_MAX_X 15.0 
#define GOAL_MIN_X -10.0

objective_t  goalkeeper_default_position(bool is_yellow);

objective_t between_goal_and_ball(bot_t goalkeeper, ball_t ball, 
                                    bool is_yellow, bool* should_hit_ball);

objective_t goalkeeper_objective(field_t* field);

#endif