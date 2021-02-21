// From: 
// https://redblobgames.github.io/circular-obstacle-pathfinding/
// https://www.redblobgames.com/pathfinding/a-star/introduction.html
// https://www.redblobgames.com/pathfinding/a-star/implementation.html
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida and Allan Cedric for Yapira UFPR

#include "path_planning.h"

// Try to add edge from circle i point P to circle j point Q
void add_edge(vector<edge_t> &surfing_edges, vector<circle_t> &circles, vector<node_t> &nodes, int i,
              float_pair P, int j, float_pair Q)
{
    if (!line_of_sight(circles, i, P, j, Q))
        return;

    // n1 = (i, P)
    node_t n1 = {.coord = {.x = P.x, .y = P.y}, .circle_index = i};
    nodes.push_back(n1); // we need to store them nodes

    // n2 = (j, Q)
    node_t n2 = {.coord = {.x = Q.x, .y = Q.y}, .circle_index = j};
    nodes.push_back(n2);

    // n1 ----- n2
    edge_t edge = {.n1 = n1, .n2 = n2};

    surfing_edges.push_back(edge); // We need to store the new edge
}

node_t circle_to_node(int circle_index, vector<node_t> &nodes)
{
    vector<node_t> nodes_on_circle;

    for (auto node : nodes)
    {
        if (node.circle_index == circle_index)
            nodes_on_circle.push_back(node);
    }

    if ((int)nodes_on_circle.size() != 1)
        cout << "start/goal should be on r=0 circle" << endl;

    return nodes_on_circle[0];
}

vector<node_t> neighbors(node_t node, vector<edge_t> &edges)
{
    vector<node_t> results;

    for (auto edge : edges)
    {
        if (edge.n1 == node)
            results.push_back(edge.n2);
        if (edge.n2 == node)
            results.push_back(edge.n1);
    }
    return results;
}

double edge_cost(node_t a, node_t b, vector<circle_t> &circles)
{
    // Adding 1 to each edge cost to favor fewer nodes in the path
    if (a.circle_index == b.circle_index)
    // If these nodes are in the same circle
    {
        // Hugging edge

        float_pair center = circles[a.circle_index].center;

        double a_angle = vec_facing(center, a.coord);
        double b_angle = vec_facing(center, b.coord);
        double delta_angle = angle_difference(a_angle, b_angle);
        return 1 + delta_angle * circles[a.circle_index].radius;
    }
    else
    {
        // surfing edge
        return 1 + vec_distance(a.coord, b.coord);
    }
}

double heuristic(node_t next, node_t goal)
{
    // We are not using yet
    (void) next;
    (void) goal;
    return 0; // TODO: not working yet
    //return vec_distance(goal_node, node);
}

Objective path(vector<fira_message::sim_to_ref::Robot> &other_robots, fira_message::sim_to_ref::Robot &my_robot,
               double x, double y, double theta)
{
    // We are not using yet
    (void)theta;

    // transforming robots into circles
    vector<circle_t> circles(other_robots.size());

    for (int i = 0; i < (int)other_robots.size(); i++)
    {
        float_pair center = {.x = other_robots[i].x(), .y = other_robots[i].y()};
        circle_t circle = {.center = center, .radius = RADIUS};
        circles.push_back(circle);
    }

    // Implementation details:
    // For special reasons we are not going to tell you,
    // we need the goal and the current position to be circles as well.
    // these circles are special, though: their radius is equal to 0.

    // start circle
    float_pair center = {.x = my_robot.x(), .y = my_robot.y()};
    circle_t start_circle = {.center = center, .radius = 0};

    // goal circle
    center = {.x = x, .y = y};
    circle_t goal_circle = {.center = center, .radius = 0};

    circles.push_back(start_circle);
    circles.push_back(goal_circle);

    // Getting the bitangents for each pair of circles
    // we are also adding every "surfing edge" (straight lines) between two nodes
    // to the vector of edges, and storing the nodes in the vector of nodes.

    vector<edge_t> surfing_edges;
    vector<node_t> nodes;

    for (int i = 0; i < (int)circles.size(); i++)
    {
        for (int j = 0; j < i; j++)
        {
            auto internal = InternalBitangents(circles[i], circles[j]);
            float_pair C = internal[0];
            float_pair D = internal[1];
            float_pair E = internal[2];
            float_pair F = internal[3];

            add_edge(surfing_edges, circles, nodes, i, C, j, F);
            if (circles[i].radius != 0 && circles[j].radius != 0)
                add_edge(surfing_edges, circles, nodes, i, D, j, E);

            auto external = ExternalBitangents(circles[i], circles[j]);
            C = external[0];
            D = external[1];
            E = external[2];
            F = external[3];

            if (circles[i].radius != 0 || circles[j].radius != 0)
                add_edge(surfing_edges, circles, nodes, i, C, j, F);

            if (circles[i].radius != 0 && circles[j].radius != 0)
                add_edge(surfing_edges, circles, nodes, i, D, j, E);
        }
    }

    // Generating hugging_edges (circle arcs)
    vector<vector<node_t>> nodes_in_each_circle((int)circles.size());

    // Get, for each circle, all the nodes it has
    for (auto node : nodes)
        nodes_in_each_circle[node.circle_index].push_back(node);

    vector<edge_t> hugging_edges;

    for (auto circle_nodes : nodes_in_each_circle)
    {
        for (int i = 0; i < (int)circle_nodes.size(); i++)
        {
            // For every circle, make an edge for each pair of nodes it has
            for (int j = 0; j < i; j++)
            {
                edge_t edge = {.n1 = circle_nodes[i], .n2 = circle_nodes[j]};
                hugging_edges.push_back(edge);
            }
        }
    }

    // Now, storing all the edges together:
    surfing_edges.insert(surfing_edges.end(), hugging_edges.begin(), hugging_edges.end());

    // A* Star
    node_t start_node = circle_to_node((int)circles.size() - 2, nodes);
    node_t goal_node = circle_to_node((int)circles.size() - 1, nodes);

    PriorityQueue<node_t, double> frontier;
    frontier.put(start_node, 0);

    map<node_t, node_t> came_from;
    map<node_t, double> cost_so_far;

    came_from[start_node] = start_node;
    cost_so_far[start_node] = 0;

    while (!frontier.empty())
    {
        node_t current = frontier.get();

        if (current == goal_node)
            break;

        for (auto next : neighbors(current, surfing_edges))
        {
            double new_cost = cost_so_far[current] + edge_cost(current, next, circles);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                came_from[next] = current;
                frontier.put(next, new_cost + heuristic(next, goal_node));
            }
        }
    }

    // Generating all path
    node_t current = goal_node;
    vector<node_t> path;
    while (!(current == start_node))
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start_node); // optional
    reverse(path.begin(), path.end());

    // We need to define an appropriate angle, but return value is kinda like this
    return Objective(path[1].coord.x, path[1].coord.y, M_PI/4.);
}