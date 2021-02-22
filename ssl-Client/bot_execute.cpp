#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

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

void PID(bot_t robot, objective_t objective, int index, bool my_robots_are_yellow, GrSim_Client *grSim_client)
{
  double Kp = 20;
  double Kd = 2.5;
  static double lastError = 0;

  double rightMotorSpeed;
  double leftMotorSpeed;

  bool reversed = false;

  double angle_rob = robot.a;

  double angle_obj = atan2(objective.y - robot.y, objective.x - robot.x);

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

void execute_bot_strats(field_t *f, GrSim_Client* commandClient)
{
    // do stuff
    // PID(...);
}