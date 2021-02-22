#ifndef HEADER
#define HEADER

#define NUM_BOTS 3

enum
{
    APROXIMA = 0,
    DECIDE_DESVIO,
    SOBE,
    DESCE,
    VOLTA
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