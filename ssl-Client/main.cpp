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

void PID(fira_message::sim_to_ref::Robot robot, Objective objective, int index, bool my_robots_are_yellow, GrSim_Client *grSim_client)
{
  double Kp = 20;
  double Kd = 2.5;
  static double lastError = 0;

  double rightMotorSpeed;
  double leftMotorSpeed;

  bool reversed = false;

  double angle_rob = robot.orientation();

  double angle_obj = atan2(objective.y() - robot.y(), objective.x() - robot.x());

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

void fill_field(fira_message::sim_to_ref::Frame detection, field_t *f)
{
  if (f->my_robots_are_yellow) {
    f->our_bots_n = detection.robots_yellow_size();
    f->their_bots_n = detection.robots_blue_size();
  } else {
    f->our_bots_n = detection.robots_blue_size();
    f->their_bots_n = detection.robots_yellow_size();
  }

  f->ball = detection.ball();
  f->ball.set_x((length + f->ball.x()) * 100);
  f->ball.set_y((width + f->ball.y()) * 100);

  for (int i = 0; i < NUM_BOTS; i++){
    if (f->my_robots_are_yellow) {
      f->our_bots[i] = detection.robots_yellow(i);
      f->their_bots[i] = detection.robots_blue(i);
    } else {
      f->our_bots[i] = detection.robots_blue(i);
      f->their_bots[i] = detection.robots_yellow(i);
    }
    f->our_bots[i].set_x((length + f->our_bots[i].x()) * 100);
    f->our_bots[i].set_y((width + f->our_bots[i].y()) * 100);
    f->our_bots[i].set_orientation(to180range(f->our_bots[i].orientation()));
    f->their_bots[i].set_x((length + f->their_bots[i].x()) * 100);
    f->their_bots[i].set_y((width + f->their_bots[i].y()) * 100);
    f->their_bots[i].set_orientation(to180range(f->their_bots[i].orientation()));
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

      //Our robot info:
      for (int i = 0; i < field.our_bots_n; i++){
        vector<fira_message::sim_to_ref::Robot> other_robots;
        for(int j = 0; j < field.our_bots_n; j++) {
          if(i != j)
            other_robots.push_back(detection.robots_blue(j));
        }
        for(int j = 0; j < field.their_bots_n; j++)
            other_robots.push_back(detection.robots_yellow(j));

        // Objective o = path(other_robots, field.our_bots[i], field.ball.x(), field.ball.y(), 0);
        // other_robots.clear();
        // PID(field.our_bots[i], o, i, field.my_robots_are_yellow, commandClient);
      }
      field_analyzer(detection, field.my_robots_are_yellow);
    } else {
      // pass and wait for wwindow
    }
  }

  return 0;
}
