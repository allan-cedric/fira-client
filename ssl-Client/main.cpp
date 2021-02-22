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

double to180range(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  }
  else if (angle > M_PI) {
    angle = angle - 2 * M_PI;
  }
  return angle;
}

double smallestAngleDiff(double target, double source)
{
  double a;
  a = fmod(target + 2 * M_PI, 2 * M_PI) - fmod(source + 2 * M_PI, 2 * M_PI);

  if (a > M_PI)
  {
    a = a - 2 * M_PI;
  }
  else if (a < -M_PI)
  {
    a = a + 2 * M_PI;
  }
  return a;
}

void PID(fira_message::sim_to_ref::Robot robot, objective_t objective, int index, bool my_robots_are_yellow, GrSim_Client *grSim_client)
{
  double Kp = 20;
  double Kd = 2.5;
  static double lastError = 0;

  double rightMotorSpeed;
  double leftMotorSpeed;

  bool reversed = false;

  double angle_rob = robot.orientation();

  double angle_obj = atan2(objective.y - robot.y(), objective.x - robot.x());

  double error = smallestAngleDiff(angle_rob, angle_obj);

  if (fabs(error) > M_PI / 2.0 + M_PI / 20.0)
  {
    reversed = true;
    angle_rob = to180range(angle_rob + M_PI);
    // Calculates the error and reverses the front of the robot
    error = smallestAngleDiff(angle_rob, angle_obj);
  }

  double motorSpeed = (Kp * error) + (Kd * (error - lastError)); // + 0.2 * sumErr;
  lastError = error;

  double baseSpeed = 30;

  // Normalize
  motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
  motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

  if (motorSpeed > 0)
  {
    leftMotorSpeed = baseSpeed;
    rightMotorSpeed = baseSpeed - motorSpeed;
  }
  else
  {
    leftMotorSpeed = baseSpeed + motorSpeed;
    rightMotorSpeed = baseSpeed;
  }

  if (reversed)
  {
    if (motorSpeed > 0)
    {
      leftMotorSpeed = -baseSpeed + motorSpeed;
      rightMotorSpeed = -baseSpeed;
    }
    else
    {
      leftMotorSpeed = -baseSpeed;
      rightMotorSpeed = -baseSpeed - motorSpeed;
    }
  }
  grSim_client->sendCommand(leftMotorSpeed, rightMotorSpeed, my_robots_are_yellow, index);
}

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

      fill_field(detection, &field);
      field_analyzer(&field);

      //Our robot info:
      for (int i = 0; i < field.our_bots_n; i++){
        vector<fira_message::sim_to_ref::Robot> other_robots;
        for(int j = 0; j < field.our_bots_n; j++) {
          if(i != j)
            other_robots.push_back(detection.robots_blue(j));
        }
        for(int j = 0; j < field.their_bots_n; j++)
            other_robots.push_back(detection.robots_yellow(j));

        // objective_t o = path(other_robots, field.our_bots[i], field.ball.x(), field.ball.y(), 0);
        // other_robots.clear();
        // PID(field.our_bots[i], o, i, field.my_robots_are_yellow, commandClient);
      }
    } else {
      // pass and wait for wwindow
    }
  }

  return 0;
}
