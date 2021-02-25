#ifndef HEADER
#define HEADER

#include "../FIRAClient/clients/referee/refereeclient.h"
#include "net/robocup_ssl_client.h"
#include "math.h"

#define NUM_BOTS 3
#define width 1.3 / 2.0f
#define length 1.7 / 2.0f

#define MID_FIELD 160/2

enum {
    GOALKEEPER = 0,
    NONE, // 1
    ATACKER, // 2
    DEFENDER, // 3
    CLOSER // 4
};

// Note that all players must stop at any FOUL, except at GAME_ON
typedef struct{
    bool
        is_kickoff,
        is_wo_kickoff, 
        is_free_ball, 
        is_goal_kick, 
        is_penalty_kick,
        is_halt, // Stop everything even processing !
        is_stop,
        is_game_on,
        is_overtime, // For group stage
        is_quadrant_1, // Top-right
        is_quadrant_2, // Top-left
        is_quadrant_3, // Bottom-left
        is_quadrant_4; // Bottom-right
}referee_flags_t;

typedef struct {
    bool
        especial_case, 
        wrc, // we are closer
        tra, // they are atacking
        wra; // we are atacking
} field_status_t;

typedef struct {
    double x, y, angle; 
} objective_t;

typedef struct {
    double x, y, a, // positions
        vx, vy, va; // speeds
    int fun; // what is th bot function now
    objective_t obj;
    int index;
    bool wants_to_hit_ball;
    double radius;
} bot_t;

typedef struct {
    double x, y, // position
        vx, vy; // speeds 
} ball_t;

typedef struct {
    int our_bots_n, their_bots_n;
    ball_t ball;
    bot_t our_bots[NUM_BOTS];
    bot_t their_bots[NUM_BOTS];
    bool my_robots_are_yellow;
    field_status_t fs;
    bot_t *closer_bot;
} field_t;

#endif