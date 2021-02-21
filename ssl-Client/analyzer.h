#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

void print_bot_info(fira_message::sim_to_ref::Robot bots[3], int color);
int field_analyzer(fira_message::sim_to_ref::Frame detection);
