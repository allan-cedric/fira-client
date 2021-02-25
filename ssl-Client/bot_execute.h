#ifndef EXECUTE
#define EXECUTE

#include "header.h"

#define BALL_RADIUS 22.5

double to180range(double);
void execute_bot_strats(field_t*, GrSim_Client*);
void PID(bot_t robot, 
        objective_t objective, 
        int index, 
        bool my_robots_are_yellow, 
        GrSim_Client *grSim_client );

#endif