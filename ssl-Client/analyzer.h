#ifndef ANALYZER
#define ANALYZER

// Uncomment to print information about all robots
#define ALL_META_INFO_ROBOTS

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

void field_analyzer(field_t*);

#endif