// Original author: Renato Sousa, 2018
//
// (Heavily) Modified by 
//                      Artur Coelho & 
//                      Gabriel Hishida & 
//                      Allan Cedric &
//                      many more on Yapira UFPR
// 2021 Yapira UFPR 
// Feel free to use and please reference us

//#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"

#include "header.h"
#include "analyzer.h"
#include "path_planning.h"
#include "bot_strategy.h"
#include "bot_execute.h"
#include "math_operations.h"

void set_bot_parametres(bot_t *a, fira_message::sim_to_ref::Robot b, int index)
{
    a->x = (length + b.x()) * 100;
    a->y = (width + b.y()) * 100;
    a->a = to180range(b.orientation());
    a->vx = b.vx();
    a->vy = b.vy();
    a->va = b.vorientation();
    a->index = index;
    a->wants_to_hit_ball = true;
    a->radius = RADIUS;
}

// fill the field struct with the frame data
void fill_field(fira_message::sim_to_ref::Frame detection, field_t *f)
{
    // number of bots
    f->our_bots_n = f->my_robots_are_yellow ? detection.robots_yellow_size() 
                                            : detection.robots_blue_size();
    f->their_bots_n = f->my_robots_are_yellow ? detection.robots_blue_size() 
                                            : detection.robots_yellow_size();

    // ball data
    fira_message::sim_to_ref::Ball ball = detection.ball();
    f->ball.x = (length + ball.x()) * 100;
    f->ball.y = (width + ball.y()) * 100;
    f->ball.vx = ball.vx();
    f->ball.vy = ball.vy();

    // ours and theirs bots data
    for (int i = 0; i < NUM_BOTS; i++){
        fira_message::sim_to_ref::Robot our_robot = f->my_robots_are_yellow 
                                                    ? detection.robots_yellow(i) 
                                                    : detection.robots_blue(i);
        fira_message::sim_to_ref::Robot their_robot = !f->my_robots_are_yellow 
                                                    ? detection.robots_yellow(i) 
                                                    : detection.robots_blue(i);

        set_bot_parametres(&f->our_bots[i], our_robot, i);
        set_bot_parametres(&f->their_bots[i], their_robot, i);
    }
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    // Field analysis
    field_t field;

    // Referee flags
    referee_flags_t referee;

    //define your team color here
    field.my_robots_are_yellow = false;

    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.0.0.1", 10002);
    visionClient->open(false);

    // Referee client
    // RefereeClient *refereeClient = new RefereeClient("224.5.23.2", 10003);

    // Command (actuator) client
    GrSim_Client *commandClient = new GrSim_Client();

    fira_message::sim_to_ref::Environment packet;

    while (true){
        // Running referee client
        // refereeClient->run();

        // referee_analyzer(refereeClient, &referee);

        if (visionClient->receive(packet) && packet.has_frame()){
            fira_message::sim_to_ref::Frame detection = packet.frame();

            // Fill field ball and bots detection data
            fill_field(detection, &field);

            // Fill field status data
            field_analyzer(&field);

            // Fill each bot objective data
            set_bot_strategies(&field); // TODO Coelho e Jimmy

            // if (game_on){
                // Executes each bot objective
                execute_bot_strats(&field, commandClient);
            // }

        } else {
            // pass and wait for window
            // stop_all();
        }
    }

    // Closing client
    // refereeClient->close();

    return 0;
}
