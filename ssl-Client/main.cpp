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

void printRobotInfo(const fira_message::sim_to_ref::Robot &robot)
{
  printf("ID=%3d \n", robot.robot_id());

  printf(" POS=<%9.2f,%9.2f> \n", robot.x(), robot.y());
  printf(" VEL=<%9.2f,%9.2f> \n", robot.vx(), robot.vy());

  printf("ANGLE=%6.3f \n", robot.orientation());
  printf("ANGLE VEL=%6.3f \n", robot.vorientation());
}

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

int estado = APROXIMA;
int estadoant;
double x, y, ang;
double x_in, y_in;

Objective defineObjectiveBlue(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball)
{
  double distancia = sqrt(pow(robot.x() - ball.x(), 2) + pow(robot.y() - ball.y(), 2));
  //calculo da distancia entre o jogador e a bola, por meio do módulo do vetor diferença!
  if (robot.x() < ball.x())
  { //se o robô está atrás da bola
    estado = APROXIMA;

    if ((robot.y() < ball.y() - 5 || robot.y() > ball.y() + 5) && robot.x() >= 22)
      //alinhando o robô com a bola!
      return Objective(ball.x() - 10, ball.y(), M_PI / 4.); // x,y,angle

    else if (distancia < 8 && 49 < robot.y() && 83 > robot.y())
    {
      //Se o robô 'está' com a bola e está indo em direção ao gol
      return Objective(ball.x(), ball.y(), M_PI / 4.);
    }
    else if (distancia < 8 && (43 > robot.y() || 83 < robot.y()))
    {
      //Se o robô 'está' com a bola e NÃO está indo em direção ao gol
      return Objective(ball.x(), 66, M_PI / 4.);
    }
    else
      return Objective(ball.x(), ball.y(), M_PI / 4.);
    //no mais, corre atrás da bola...
  }
  else
  {
    switch (estado)
    {
    case APROXIMA:
      //corre atras da bola ate chegar perto
      if (distancia < 10)
        estado = DECIDE_DESVIO;
      x = ball.x() + 10;
      y = ball.y();
      ang = 0; // x,y,angle
      break;

    case DECIDE_DESVIO:
      //desvia da bola, para voltar a ficar atrás dela
      if (robot.y() - 10 > 6)
        estado = SOBE;
      else
        estado = DESCE;

      break;

    case SOBE:
      estadoant = SOBE;
      estado = VOLTA;
      x = ball.x() + 10;
      y = ball.y() - 8;
      x_in = robot.x();
      y_in = robot.y();
      ang = M_PI / 2.0; // x,y,angle

      break;
    case DESCE:
      estadoant = DESCE;
      estado = VOLTA;
      x = ball.x() + 10;
      y = ball.y() + 8;
      x_in = robot.x();
      y_in = robot.y();
      ang = M_PI / 2.0; // x,y,angle

      break;
    case VOLTA:
      if (robot.x() <= x_in - 16 || robot.x() <= 12)
      {
        //se ele andou 16 ou tá no limite do mapa
        //fazendo_manobra = 0;
        estado = APROXIMA;
      }
      if (estadoant == DESCE)
      {
        x = ball.x() - 16;
        y = ball.y() + 8;
        ang = M_PI; // x,y,angle */
      }
      else
      {
        x = ball.x() - 16;
        y = ball.y() - 8;
        ang = M_PI; // x,y,angle */
      }
      break;
    }

    return Objective(x, y, ang);
  }
}

Objective defineObjectiveYellow(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball)
{
  double distancia = sqrt(pow(robot.x() - ball.x(), 2) + pow(robot.y() - ball.y(), 2));
  //calculo da distancia entre o jogador e a bola, por meio do módulo do vetor diferença!
  if (robot.x() > ball.x())
  { //se o robô está atrás da bola
    estado = APROXIMA;

    if ((robot.y() > ball.y() - 5 || robot.y() < ball.y() + 5) && robot.x() <= 22)
      //alinhando o robô com a bola!
      return Objective(ball.x() - 10, ball.y(), M_PI / 4.); // x,y,angle

    else if (distancia < 8 && 49 > robot.y() && 83 < robot.y())
    {
      //Se o robô 'está' com a bola e está indo em direção ao gol
      return Objective(ball.x(), ball.y(), M_PI / 4.);
    }
    else if (distancia < 8 && (43 < robot.y() || 83 > robot.y()))
    {
      //Se o robô 'está' com a bola e NÃO está indo em direção ao gol
      return Objective(ball.x(), 66, M_PI / 4.);
    }
    else
      return Objective(ball.x(), ball.y(), M_PI / 4.);
    //no mais, corre atrás da bola...
  }
  else
  {
    switch (estado)
    {
    case APROXIMA:
      //corre atras da bola ate chegar perto
      if (distancia < 10)
        estado = DECIDE_DESVIO;
      x = ball.x() + 10;
      y = ball.y();
      ang = 0; // x,y,angle
      break;

    case DECIDE_DESVIO:
      //desvia da bola, para voltar a ficar atrás dela
      if (robot.y() - 10 < 6)
        estado = SOBE;
      else
        estado = DESCE;

      break;

    case SOBE:
      estadoant = SOBE;
      estado = VOLTA;
      x = ball.x() + 10;
      y = ball.y() - 8;
      x_in = robot.x();
      y_in = robot.y();
      ang = M_PI / 2.0; // x,y,angle

      break;
    case DESCE:
      estadoant = DESCE;
      estado = VOLTA;
      x = ball.x() + 10;
      y = ball.y() + 8;
      x_in = robot.x();
      y_in = robot.y();
      ang = M_PI / 2.0; // x,y,angle

      break;
    case VOLTA:
      if (robot.x() >= x_in - 16 || robot.x() >= 12)
      {
        //se ele andou 16 ou tá no limite do mapa
        //fazendo_manobra = 0;
        estado = APROXIMA;
      }
      if (estadoant == DESCE)
      {
        x = ball.x() - 16;
        y = ball.y() + 8;
        ang = M_PI; // x,y,angle */
      }
      else
      {
        x = ball.x() - 16;
        y = ball.y() - 8;
        ang = M_PI; // x,y,angle */
      }
      break;
    }

    return Objective(x, y, ang);
  }
}

int main(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  //define your team color here
  bool my_robots_are_yellow = false;

  // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
  RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.0.0.1", 10002);
  visionClient->open(false);

  GrSim_Client *commandClient = new GrSim_Client();

  fira_message::sim_to_ref::Environment packet;

  while (true){
    if (visionClient->receive(packet) && packet.has_frame()){
      fira_message::sim_to_ref::Frame detection = packet.frame();

      int robots_blue_n = detection.robots_blue_size();
      int robots_yellow_n = detection.robots_yellow_size();
      double width, length;
      width = 1.3 / 2.0;
      length = 1.7 / 2.0;

      //Ball info:

      fira_message::sim_to_ref::Ball ball = detection.ball();
      ball.set_x((length + ball.x()) * 100);
      ball.set_y((width + ball.y()) * 100);
      printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());
      
      // Path planning test
      //Blue robot info:
      //for (int i = 0; i < robots_blue_n; i++){
        int i = 0;
        fira_message::sim_to_ref::Robot robot_B = detection.robots_blue(i);
        robot_B.set_x((length + robot_B.x()) * 100); //convertendo para centimetros
        robot_B.set_y((width + robot_B.y()) * 100);
        robot_B.set_orientation(to180range(robot_B.orientation()));
        // printf("-Robot(B) (%2d/%2d): ", i + 1, robots_blue_n);
        // printRobotInfo(robot_B);

        //Objective o = defineObjectiveBlue(robot_B, ball);
        vector<fira_message::sim_to_ref::Robot> other_robots;
        for(int j = 0; j < robots_blue_n; j++)
        {
          if(i != j)
            other_robots.push_back(detection.robots_blue(j));
        }
        for(int j = 0; j < robots_yellow_n; j++)
            other_robots.push_back(detection.robots_yellow(j));

        Objective o = path(other_robots, robot_B, ball.x(), ball.y(), 0);
        PID(robot_B, o, i, my_robots_are_yellow, commandClient);
      //}

      //Yellow robot info:
      /*for (int i = 0; i < robots_yellow_n; i++){
        fira_message::sim_to_ref::Robot robot_Y = detection.robots_yellow(i);
        robot_Y.set_x((length + robot_Y.x()) * 100); //convertendo para centimetros
        robot_Y.set_y((width + robot_Y.y()) * 100);
        robot_Y.set_orientation(to180range(robot_Y.orientation()));
        // printf("-Robot(Y) (%2d/%2d): ", i + 1, robots_yellow_n);
        // printRobotInfo(robot_Y);

        Objective o = defineObjectiveYellow(robot_Y, ball);
        PID(robot_Y, o, i, !my_robots_are_yellow, commandClient);
      }*/
      field_analyzer(detection);
    } else {
      // pass
    }
  }

  return 0;
}
