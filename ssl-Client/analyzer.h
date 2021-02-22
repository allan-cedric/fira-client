#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

void print_bot_info(fira_message::sim_to_ref::Robot[3], int);
int field_analyzer(field_t*);
