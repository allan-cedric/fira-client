// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida for Yapira UFPR

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;


typedef struct {
    double x,y;
} float_pair;

#define RADIUS 8.0

class circle_t {
    public:
        float_pair center;
        float radius = 8.0;
        circle_t(float_pair center);
};

circle_t::circle_t(float_pair center)
{
    this->center = center;
    this->radius = RADIUS;
}

// from lib.js:


float_pair vec_polar(double r, double a) {
    float_pair polar;
    polar.x = r * cos(a);
    polar.y = r * sin(a);
    return polar;
}

float_pair vec_add(float_pair p, float_pair q) {
    float_pair sum;
    sum.x = p.x + q.x;
    sum.y = p.y + q.y;
    return sum;
}

float_pair vec_sub(float_pair p, float_pair q) {
    float_pair diff;
    diff.x = p.x + q.x;
    diff.y = p.y + q.y;
    return diff;
}

double vec_dot(float_pair p, float_pair q) {
    return (p.x * q.x + p.y * q.y);
}

double vec_cross(float_pair p, float_pair q) {
    return p.x * q.y - p.y * q.x;
}

float_pair vec_interpolate(float_pair p, float_pair q, double t) {
    float_pair interpolation;
    interpolation.x = p.x + (q.x - p.x) * t;
    interpolation.y = p.y + (q.y - p.y) * t;

    return interpolation;
}

double vec_facing(float_pair p, float_pair q) {
    double dx = q.x - p.x;
    double dy = q.y - p.y;

    return atan2(dy, dx);
}

double vec_length(float_pair p) {
    return sqrt(p.x*p.x + p.y*p.y);
}

double vec_distance(float_pair p, float_pair q) {
    return vec_length(vec_sub(p, q));
}

float_pair vec_normalize(float_pair p) {
    double length = vec_length(p);
    double d = length?length:1e-6; // avoid divide by 0
    
    float_pair normalized;
    normalized.x = p.x / d;
    normalized.y = p.y / d;

    return normalized;
}

double angle_difference(double a, double b) {
    return fmod(fabs(b - a), (2 * M_PI));
}

// from belt_problem.js:

float_pair direction_step(float_pair start, double distance, double angle) {
    return vec_add(start, vec_polar(distance, angle));
}

vector<float_pair> InternalBitangents(circle_t A, circle_t B)
{
    double P = vec_distance(A.center, B.center);
    double cos_angle = (A.radius + B.radius) / P;
    double theta = acos(cos_angle);

    double AB_angle = vec_facing(A.center, B.center); 
    double BA_angle = vec_facing(B.center, A.center); 
        
    float_pair C = direction_step(A.center, A.radius, AB_angle - theta);
    float_pair D = direction_step(A.center, A.radius, AB_angle + theta);
    float_pair E = direction_step(B.center, B.radius, BA_angle + theta);
    float_pair F = direction_step(B.center, B.radius, BA_angle - theta);

    vector<float_pair> CEDF;
    CEDF.push_back(C);
    CEDF.push_back(E);
    CEDF.push_back(D);
    CEDF.push_back(F);

    return CEDF;
}

vector<float_pair> ExternalBitangents(circle_t A, circle_t B)
{
    double P = vec_distance(A.center, B.center);
    double cos_angle = (A.radius - B.radius) / P;
    double theta = acos(cos_angle);

    double AB_angle = vec_facing(A.center, B.center); 
        
    float_pair C = direction_step(A.center, A.radius, AB_angle - theta);
    float_pair D = direction_step(A.center, A.radius, AB_angle + theta);
    float_pair E = direction_step(B.center, B.radius, AB_angle + theta);
    float_pair F = direction_step(B.center, B.radius, AB_angle - theta);

    vector<float_pair> CEDF;
    CEDF.push_back(C);
    CEDF.push_back(E);
    CEDF.push_back(D);
    CEDF.push_back(F);

    return CEDF;
}
