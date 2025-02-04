#ifndef ANALYZER
#define ANALYZER

// Uncomment to print information about all robots
#define ALL_META_INFO_ROBOTS

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

void print_bot_info(bot_t bots[NUM_BOTS]);
void print_ball_info(ball_t ball);
void print_field(field_t *f);
int max_dist_index(double d[NUM_BOTS]);
int min_dist_index(double d[NUM_BOTS]);
bool is_on_my_field(double x, bool mray);
bool we_are_closer(field_t *f);
bool they_are_atacking(field_t *f);
bool we_are_atacking(field_t *f);
void field_analyzer(field_t*);
void referee_analyzer(RefereeClient *, referee_flags_t *);

#endif
