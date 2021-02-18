#ifndef PATH_PLANNING
#define PATH_PLANNING

#include "math_operations.h"
#include "net/robocup_ssl_client.h"
#include "header.h"
#include <map>

void add_edge(vector<edge_t> &surfing_edges, vector<circle_t> circles, vector<node_t> &nodes, int i,
              float_pair P, int j, float_pair Q);

node_t circle_to_node(int circle_index, vector<node_t> nodes);

bool node_comparison(node_t N, node_t M);

vector<node_t> neighbors(node_t node, vector<edge_t> edges);

double edge_cost(node_t a, node_t b, vector<circle_t> circles);

double heuristic(node_t node);

Objective path(vector<fira_message::sim_to_ref::Robot &> other_robots, fira_message::sim_to_ref::Robot &my_robot, 
               double x, double y, double theta);

#endif