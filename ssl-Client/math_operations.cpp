// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida for Yapira UFPR

#include "math_operations.h"

// from lib.js:

float_pair_t vec_polar(double r, double a)
{
    float_pair_t polar;
    polar.x = r * cos(a);
    polar.y = r * sin(a);
    return polar;
}

float_pair_t vec_add(float_pair_t p, float_pair_t q)
{
    float_pair_t sum;
    sum.x = p.x + q.x;
    sum.y = p.y + q.y;
    return sum;
}

float_pair_t vec_sub(float_pair_t p, float_pair_t q)
{
    float_pair_t diff;
    diff.x = p.x - q.x;
    diff.y = p.y - q.y;
    return diff;
}

double vec_dot(float_pair_t p, float_pair_t q)
{
    return (p.x * q.x + p.y * q.y);
}

double vec_cross(float_pair_t p, float_pair_t q)
{
    return p.x * q.y - p.y * q.x;
}

float_pair_t vec_interpolate(float_pair_t p, float_pair_t q, double t)
{
    float_pair_t interpolation;
    interpolation.x = p.x + (q.x - p.x) * t;
    interpolation.y = p.y + (q.y - p.y) * t;

    return interpolation;
}

double vec_facing(float_pair_t p, float_pair_t q)
{
    double dx = q.x - p.x;
    double dy = q.y - p.y;

    return atan2(dy, dx);
}

double vec_length(float_pair_t p)
{
    return sqrt(p.x * p.x + p.y * p.y);
}

double vec_distance(float_pair_t p, float_pair_t q)
{
    return vec_length(vec_sub(p, q));
}

float_pair_t vec_normalize(float_pair_t p)
{
    double length = vec_length(p);
    double d = length ? length : 1e-6; // avoid divi, P, j, Q) {ide by 0

    float_pair_t normalized;
    normalized.x = p.x / d;
    normalized.y = p.y / d;

    return normalized;
}

double angle_difference(double a, double b)
{
    return fmod(fabs(b - a), (2 * M_PI));
}

// from belt_problem.js:

float_pair_t direction_step(float_pair_t start, double distance, double angle)
{
    return vec_add(start, vec_polar(distance, angle));
}

vector<float_pair_t> InternalBitangents(circle_t A, circle_t B)
{
    double P = vec_distance(A.center, B.center);
    double cos_angle = (A.radius + B.radius) / P;

    if (cos_angle > 1) // circles overlap, there are no internal bitangents
        return vector<float_pair_t>();

    double theta = acos(cos_angle);

    double AB_angle = vec_facing(A.center, B.center);
    double BA_angle = vec_facing(B.center, A.center);

    float_pair_t C = direction_step(A.center, A.radius, AB_angle - theta);
    float_pair_t D = direction_step(A.center, A.radius, AB_angle + theta);
    float_pair_t E = direction_step(B.center, B.radius, BA_angle + theta);
    float_pair_t F = direction_step(B.center, B.radius, BA_angle - theta);

    vector<float_pair_t> CDEF;
    CDEF.push_back(C);
    CDEF.push_back(D);
    CDEF.push_back(E);
    CDEF.push_back(F);

    return CDEF;
}

vector<float_pair_t> ExternalBitangents(circle_t A, circle_t B)
{
    double P = vec_distance(A.center, B.center);
    double cos_angle = fabs(A.radius - B.radius) / P;
    double theta = acos(cos_angle);

    // Circle inside another, no external bitangents.
    // if (!(cos_angle < 1 || cos_angle > -1))
    //     return vector<float_pair_t>();

    double AB_angle = vec_facing(A.center, B.center);

    float_pair_t C = direction_step(A.center, A.radius, AB_angle - theta);
    float_pair_t D = direction_step(A.center, A.radius, AB_angle + theta);
    float_pair_t E = direction_step(B.center, B.radius, AB_angle + theta);
    float_pair_t F = direction_step(B.center, B.radius, AB_angle - theta);

    vector<float_pair_t> CDEF;
    CDEF.push_back(C);
    CDEF.push_back(D);
    CDEF.push_back(E);
    CDEF.push_back(F);

    return CDEF;
}

bool segment_circle_intersection(float_pair_t A, float_pair_t B, circle_t C)
{
    if(C.radius <= 0)
        return false;

    float_pair_t CA = vec_sub(C.center, A), BA = vec_sub(B, A);

    double u = (CA.x * BA.x + CA.y * BA.y) / (BA.x * BA.x + BA.y * BA.y);

    if (u < 0.0)
        u = 0.0;
    if (u > 1.0)
        u = 1.0;

    float_pair_t E = vec_interpolate(A, B, u);
    double d = vec_distance(C.center, E);

    return (d < C.radius);
}

bool line_of_sight(vector<circle_t> &circles, int i, float_pair_t P, int j, float_pair_t Q)
{
    for (int k = 0; k < (int)circles.size(); k++)
    {
        if (k != i && k != j && segment_circle_intersection(P, Q, circles[k]))
           return false;
        // else if (k == i || k == j)
        // {
        //     if()   
        // }
    }
    return true;
}

// I know it's not efficient, but it may not be matter.
bool is_blocking(float_pair_t D, float_pair_t E, float_pair_t I, float_pair_t J, float_pair_t A)
{
    A = {.x = 0, .y = 0};
    D = {.x = 1, .y = 1};
    E = {.x = 1, .y = -1};

    I = {.x = 1, .y = 1.5};
    J = {.x = 1, .y = -1.5};

    double theta_D = atan2(D.y - A.y, D.x - A.x);

    if (theta_D < 0)
        theta_D += 2 * M_PI;

    double theta_E = atan2(E.y - A.y, E.x - A.x);

    if (theta_E < 0)
        theta_E += 2 * M_PI;

    double theta_I = atan2(I.y - A.y, I.x - A.x);

    if (theta_I < 0)
        theta_I += 2 * M_PI;

    double theta_J = atan2(J.y - A.y, J.x - A.x);

    if (theta_J < 0)
        theta_J += 2 * M_PI;

    bool discard = false;

    double gamma_1 = max(theta_J, theta_I) - min(theta_J, theta_I);
    double gamma_2 = min(theta_J, theta_I) + 2 * M_PI - max(theta_J, theta_I);

    // A entrada ou a saída em intervalo proibido
    /*if (((theta_I > min(theta_E, theta_D) && theta_I < max(theta_E, theta_D) && (theta_J > max(theta_E, theta_D) || theta_J < min(theta_E, theta_D))))
     || ((theta_J > min(theta_E, theta_D) && theta_J < max(theta_E, theta_D) && (theta_I > max(theta_E, theta_D) || theta_I < min(theta_E, theta_D)))))*/
    if ((inrange(min(theta_E, theta_D), max(theta_E, theta_D), theta_I) && !inrange(min(theta_E, theta_D), max(theta_E, theta_D), theta_J)) || (inrange(min(theta_E, theta_D), max(theta_E, theta_D), theta_J) && !inrange(min(theta_E, theta_D), max(theta_E, theta_D), theta_I)))
        discard = true;
    else if (gamma_1 > gamma_2)
    {
        // Nem a saída e nem a entrada em intervalo proibido, mas na passagem de um pro outro atravessa o intervalo proibido.
        // (Anti-horário)
        if (inrange(min(theta_I, theta_J), min(theta_I, theta_J) + gamma_1, theta_D))
            discard = true;
    }
    // Nem a saída e nem a entrada em intervalo proibido, mas na passagem de um pro outro atravessa o intervalo proibido.
    // (Horário)
    else if (!inrange(min(theta_I, theta_J), min(theta_I, theta_J) + gamma_1, theta_D))
        discard = true;

    return discard;
}

bool inrange(double a, double b, double to_test)
{
    return (to_test >= a && to_test <= b);
}

bool is_blocking_js(circle_t A, circle_t B)
{
    double AB_distance = vec_distance(A.center, B.center);
    double AB_angle = vec_facing(A.center, B.center);

    double a = AB_distance / 2;
    double theta = acos(a / A.radius);
    
    float_pair_t D = direction_step(A.center, A.radius, AB_angle + theta);
    float_pair_t E = direction_step(A.center, A.radius, AB_angle - theta);

    double AD_angle = vec_facing(A.center, D);
    double AE_angle = vec_facing(A.center, E);

    return (AD_angle < 0 || AE_angle < 0);
}

bool node_found(node_t node)
{
    return (!isnan(node.coord.x) && !isnan(node.coord.y));
}
