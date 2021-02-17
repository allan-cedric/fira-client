// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida for Yapira UFPR

#define _USE_MATH_DEFINES
#include <cmath>

typedef struct {
    double x,y;
} float_pair;


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
