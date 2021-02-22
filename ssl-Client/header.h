#ifndef HEADER
#define HEADER

#include "net/robocup_ssl_client.h"

#define NUM_BOTS 3
#define width 1.3 / 2.0f
#define length 1.7 / 2.0f

#define MID_FIELD 160/2

enum {
    GOALKEEPER = 0,
    ATACKER,
    DEFENDER
};

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
} field_t;

#endif