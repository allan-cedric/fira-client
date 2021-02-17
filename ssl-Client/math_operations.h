// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida for Yapira UFPR

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;

typedef struct
{
    double x, y;
} float_pair;

#define RADIUS 8.0

class circle_t
{
public:
    float_pair center;
    float radius = 8.0;
    circle_t(float_pair center)
    {
        this->center = center;
        this->radius = RADIUS;
    }
};

typedef struct{
    float_pair coord;
    int circle_index;
} node_t;

typedef struct {
    node_t n1, n2;
} edge_t;


// from lib.js:

float_pair vec_polar(double r, double a);

float_pair vec_add(float_pair p, float_pair q);

float_pair vec_sub(float_pair p, float_pair q);

double vec_dot(float_pair p, float_pair q);

double vec_cross(float_pair p, float_pair q);

float_pair vec_interpolate(float_pair p, float_pair q, double t);

double vec_facing(float_pair p, float_pair q);

double vec_length(float_pair p);

double vec_distance(float_pair p, float_pair q);

float_pair vec_normalize(float_pair p);

double angle_difference(double a, double b);

// from belt_problem.js:

float_pair direction_step(float_pair start, double distance, double angle);

vector<float_pair> InternalBitangents(circle_t A, circle_t B);

vector<float_pair> ExternalBitangents(circle_t A, circle_t B);

bool segment_circle_intersection(float_pair A, float_pair B, circle_t C);

bool line_of_sight(vector<circle_t> circles, int i, float_pair P, int j, float_pair Q);