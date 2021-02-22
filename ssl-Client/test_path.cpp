//author  Renato Sousa, 2018
//#include <QtNetwork>
#include <stdio.h>
#include <iostream>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"
#include "analyzer.h"
#include "path_planning.h"
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
  bool one_time = true;

  // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
  RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.0.0.1", 10002);
  visionClient->open(false);

  GrSim_Client *commandClient = new GrSim_Client();

  fira_message::sim_to_ref::Environment packet;

  while (true){
    if (visionClient->receive(packet) && packet.has_frame()){
      fira_message::sim_to_ref::Frame detection = packet.frame();

      fill_field(detection, &field);
      field_analyzer(&field);

      //Our robot info:
      int i = 0;
      //for (int i = 0; i < field.our_bots_n - field.our_bots_n + 1; i++){
        
        bot_t other_robots[field.our_bots_n + field.their_bots_n - 1];

        for(int a = 0, b = 0; a < field.our_bots_n + field.their_bots_n - 1; b++)
        {
          if(b != i)
            other_robots[a++] = field.our_bots[b];
          other_robots[a++] = field.their_bots[b];
        }
        
        if(one_time)
          objective_t o = path(other_robots, &field, field.our_bots[i], field.ball.x, field.ball.y, 0);
        one_time = false;
        // PID(field.our_bots[i], o, i, field.my_robots_are_yellow, commandClient);
      //}
    } else {
      // pass and wait for wwindow
    }
  }

  return 0;
}
