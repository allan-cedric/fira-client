// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida and Allan Cedric for Yapira UFPR

#ifndef MATH_OPERATIONS
#define MATH_OPERATIONS

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;

struct float_pair_t {
    double x, y;
};

#define RADIUS 13.0
#define MIN_HUGGING_ANGLE M_PI/6
#define HUGGING_EDGE_STEPS 3


struct circle_t
{
    float_pair_t center;
    double radius;
};

struct node_t
{
    float_pair_t coord;
    int circle_index;

    bool operator<(const node_t &t) const
    {
        return (this->coord.x < t.coord.x ||
                (this->coord.x == t.coord.x && this->coord.y < t.coord.y));
    }

    bool operator==(const node_t &t) const
    {
        if (this->circle_index == t.circle_index)
        {
            if (this->coord.x == t.coord.x && this->coord.y == t.coord.y)
                return true;
        }

        return false;
    }
};

struct edge_t
{
    node_t n1, n2;
};

// from lib.js:

float_pair_t vec_polar(double r, double a);

float_pair_t vec_add(float_pair_t p, float_pair_t q);

float_pair_t vec_sub(float_pair_t p, float_pair_t q);

double vec_dot(float_pair_t p, float_pair_t q);

double vec_cross(float_pair_t p, float_pair_t q);

float_pair_t vec_interpolate(float_pair_t p, float_pair_t q, double t);

double vec_facing(float_pair_t p, float_pair_t q);

double vec_length(float_pair_t p);

double vec_distance(float_pair_t p, float_pair_t q);

float_pair_t vec_normalize(float_pair_t p);

double angle_difference(double a, double b);

// from belt_problem.js:

float_pair_t direction_step(float_pair_t start, double distance, double angle);

vector<float_pair_t> InternalBitangents(circle_t A, circle_t B);

vector<float_pair_t> ExternalBitangents(circle_t A, circle_t B);

bool segment_circle_intersection(float_pair_t A, float_pair_t B, circle_t C);

bool line_of_sight(vector<circle_t> &circles, int i, float_pair_t P, int j, float_pair_t Q);

bool is_blocking(float_pair_t D, float_pair_t E, float_pair_t I, float_pair_t J, float_pair_t A);

bool inrange(double a, double b, double to_test);

bool is_blocking_js(circle_t A, circle_t B);

bool node_found(node_t node);



#endif