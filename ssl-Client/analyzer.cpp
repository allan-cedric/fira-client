#include <stdio.h>
#include <stdlib.h>

#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"

int estado = APROXIMA;
int estadoant;
double x, y, ang;
double x_in, y_in;

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

int main()
{

    return 0;
}