#include "header.h"

void Objective::setY(double value)
{
    m_y = value;
}

void Objective::setAngle(double value)
{
    m_angle = value;
}

void Objective::setX(double value)
{
    m_x = value;
}

double Objective::x()
{
    return m_x;
}

double Objective::y()
{
    return m_y;
}

double Objective::angle()
{
    return m_angle;
}