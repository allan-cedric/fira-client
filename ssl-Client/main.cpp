//author  Renato Sousa, 2018
// (Heavly) Modified by Artur Coelho, Gabriel Hishida & Allan Cedric on Feb 2021
// For Yapira UFPR 

//#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"

#include "header.h"
#include "analyzer.h"
#include "path_planning.h"
#include "bot_strategy.h"
#include "bot_execute.h"

void set_bot_parametres(bot_t *a, fira_message::sim_to_ref::Robot b)
{
    a->x = (length + b.x()) * 100;
    a->y = (width + b.y()) * 100;
    a->a = to180range(b.orientation());
    a->vx = b.vx();
    a->vy = b.vy();
    a->va = b.vorientation();
}

// fill the field struct with the frame data
void fill_field(fira_message::sim_to_ref::Frame detection, field_t *f)
{
    // number of bots
    f->our_bots_n = f->my_robots_are_yellow ? detection.robots_yellow_size() : detection.robots_blue_size();
    f->their_bots_n = f->my_robots_are_yellow ? detection.robots_blue_size() : detection.robots_yellow_size();

    // ball data
    fira_message::sim_to_ref::Ball ball = detection.ball();
    f->ball.x = (length + ball.x()) * 100;
    f->ball.y = (width + ball.y()) * 100;
    f->ball.vx = ball.vx();
    f->ball.vy = ball.vy();

    // ours and theirs bots data
    for (int i = 0; i < NUM_BOTS; i++){
        fira_message::sim_to_ref::Robot our_robot = f->my_robots_are_yellow ? detection.robots_yellow(i) : detection.robots_blue(i);
        fira_message::sim_to_ref::Robot their_robot = !f->my_robots_are_yellow ? detection.robots_yellow(i) : detection.robots_blue(i);

        set_bot_parametres(&f->our_bots[i], our_robot);
        set_bot_parametres(&f->their_bots[i], their_robot);
    }
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    field_t field;

    //define your team color here
    field.my_robots_are_yellow = false;

    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.0.0.1", 10002);
    visionClient->open(false);

    GrSim_Client *commandClient = new GrSim_Client();

    fira_message::sim_to_ref::Environment packet;

    while (true){
        if (visionClient->receive(packet) && packet.has_frame()){
            fira_message::sim_to_ref::Frame detection = packet.frame();

            // Fill field ball and bots detection data
            fill_field(detection, &field);

            // Fill field status data
            field_analyzer(&field); // TO COMPLETE Allan e TJ
            // extra_cases(); // faltas, tiros de meta, penaltis -> Allan e TJ

            // Fill each bot objective data
            set_bot_strategies(&field); // TODO Coelho e Jimmy
            // set_goalkeeper_strategy(&field); // Hishida e Resenha

            // Executes each bot objective
            execute_bot_strats(&field, commandClient); // Allan e Xandão

        } else {
            // pass and wait for window
        }
    }

    return 0;
}
