#ifndef EXECUTE
#define EXECUTE

#include "net/grSim_client.h"
#include "header.h"

double to180range(double);
void execute_bot_strats(field_t*, GrSim_Client*);
void PID(bot_t robot, 
        objective_t objective, 
        int index, 
        bool my_robots_are_yellow, 
        GrSim_Client *grSim_client );

#endif