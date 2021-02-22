#ifndef HEADER
#define HEADER

#include "net/robocup_ssl_client.h"

#define NUM_BOTS 3
#define width 1.3 / 2.0f
#define length 1.7 / 2.0f

#define MID_FIELD 160/2

typedef struct {
    double x, y, vx, vy;
} bot_t;

typedef struct {
    int our_bots_n, their_bots_n;
    fira_message::sim_to_ref::Ball ball;
    fira_message::sim_to_ref::Robot our_bots[NUM_BOTS];
    fira_message::sim_to_ref::Robot their_bots[NUM_BOTS];
    bool my_robots_are_yellow;
} field_t;

// field status
enum
{
    WRC = 1, // we are closer
    TRC, // they are closer
    WRA, // we are atacking
    TRA, // they are atacking
};

class Objective
{
    double m_x;
    double m_y;
    double m_angle;

    public:
        Objective(double t_x, double t_y, double t_angle) : m_x(t_x), m_y(t_y), m_angle(t_angle){};

        void setY(double value);
        void setAngle(double value);
        void setX(double value);
        double x();
        double y();
        double angle();
};

#endif