// From https://redblobgames.github.io/circular-obstacle-pathfinding/
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida for Yapira UFPR

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;

struct float_pair
{
    double x, y;
};

#define RADIUS 8.0

struct circle_t
{
    float_pair center;
    float radius;
};

struct node_t
{
    float_pair coord;
    int circle_index;

    // We use this data type to be a key in a map. So, we have to overwrite.
    bool operator<(const node_t& t) const
    {
        return (this->coord.x < t.coord.x || 
        (this->coord.x == t.coord.x && this->coord.y < t.coord.y));
    }
};

struct edge_t
{
    node_t n1, n2;
};

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