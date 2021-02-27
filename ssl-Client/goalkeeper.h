#ifndef GOALKEEPER
#define GOALKEEPER




#include "header.h"
#include "bot_strategy.h"

#define FIELD_LENGTH 160.0

// field.height/2 == 65
// goalkeeper default position
#define GK_DEFAULT_X 25.0
#define GK_DEFAULT_Y 65.0

// goal area:
#define GOAL_MAX_Y 85 //65 + 20
#define GOAL_MIN_Y 45 //65 - 20
#define GOAL_MAX_X 15.0 
#define GOAL_MIN_X -10.0

// if the goalkeeper is closer than this, it should kick the ball away
#define GK_BALL_DIST 8.0

// if the goalkeeper is this far away from the goal, it should stop following the ball
#define GK_GOAL_DIST 40.0


objective_t  goalkeeper_default_position(bool is_yellow);

objective_t between_goal_and_ball(bot_t goalkeeper, ball_t ball, 
                                    bool is_yellow, bool* should_hit_ball);

objective_t goalkeeper_objective(field_t* field);

#endif