// From:
// https://redblobgames.github.io/circular-obstacle-pathfinding/
// https://www.redblobgames.com/pathfinding/a-star/introduction.html
// https://www.redblobgames.com/pathfinding/a-star/implementation.html
// Copyright 2017 Red Blob Games <redblobgames@gmail.com>
// License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

// Translated to C++ by Gabriel Hishida and Allan Cedric for Yapira UFPR

#include "header.h"
#include "path_planning.h"

vector<objective_t> intermediate_steps(float_pair start, float_pair end, circle_t circle)
// in a hugging edge, get intermediate steps as objectives in order to run around the circle
{
    float_pair center = circle.center;
    int number_of_steps = HUGGING_EDGE_STEPS;

    double theta_start = atan2(start.y - center.y, start.x - center.x);
    
    if (theta_start < 0)
        theta_start += 2 * M_PI;

    double theta_end = atan2(end.y - center.y, end.x - center.x);
    if (theta_end < 0)
        theta_end += 2 * M_PI;

    double PHI = theta_end - theta_start;

    //acha o sentido de rotação pelo menor arco de circunferência
    double gamma_1 = max(theta_end, theta_start) - min(theta_end, theta_start);
    double gamma_2 = min(theta_end, theta_start) + 2 * M_PI - max(theta_end, theta_start);

    vector<objective_t> intermediate_points;
    objective_t obj;

    if (PHI < MIN_HUGGING_ANGLE) // distance too small, no need for intermediate step
    {
        obj.x = end.x;
        obj.y = end.y;

        if (gamma_1 < gamma_2) // anticlockwise
            obj.angle = theta_end + M_PI/2;
        else
            obj.angle = theta_end - M_PI/2;

        intermediate_points.push_back(obj);
    }
    else // enough distance for an intermediate
    {

        float_pair intermediate_point;
        double angle;
 
        for(int i = 1; i < number_of_steps; i++)
        {

            if (gamma_1 < gamma_2) //anticlockwise
            {
                intermediate_point = vec_add(vec_polar(circle.radius,theta_start + (PHI/number_of_steps) * i), center);
                angle = theta_start + (PHI/number_of_steps) * i + M_PI/2;
            }
            else //clockwise
            {
                intermediate_point = vec_add(vec_polar(circle.radius,theta_start - (PHI/number_of_steps) * i), center);
                angle = theta_start + (PHI/number_of_steps) * i - M_PI/2;
            }

            obj.x = intermediate_point.x;
            obj.y = intermediate_point.y;
            obj.angle = angle;

            intermediate_points.push_back(obj);
        }
    }

    return intermediate_points;
}


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

    /*if ((int)nodes_on_circle.size() != 1)
        cout << "start/goal should be on r=0 circle" << endl;*/

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
    (void)next;
    (void)goal;
    return 0; // TODO: not working yet
    //return vec_distance(goal_node, node);
}

objective_t path(bot_t *other_robots, field_t *field, bot_t my_robot,
               double x, double y, double theta)
{

    // transforming robots into circles
    vector<circle_t> circles;

    for (int i = 0; i < field->their_bots_n + field->our_bots_n - 1; i++)
    {
        float_pair center = {.x = other_robots[i].x, .y = other_robots[i].y};
        circle_t circle = {.center = center, .radius = RADIUS};
        circles.push_back(circle);
    }

    // Implementation details:
    // For special reasons we are not going to tell you,
    // we need the goal and the current position to be circles as well.
    // these circles are special, though: their radius is equal to 0.

    // start circle
    float_pair center = {.x = my_robot.x, .y = my_robot.y};
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
            float_pair C, D, E, F;

            auto internal = InternalBitangents(circles[i], circles[j]);

            if (!internal.empty()) // there are internal bitangents, circles don't overlap
            {
                C = internal[0];
                D = internal[1];
                E = internal[2];
                F = internal[3];

                add_edge(surfing_edges, circles, nodes, i, C, j, F);
                if (circles[i].radius != 0 && circles[j].radius != 0)
                    add_edge(surfing_edges, circles, nodes, i, D, j, E);
            }

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
    bool blocking_edge;

    for (auto circle_nodes : nodes_in_each_circle)
    {
        for (int i = 0; i < (int)circle_nodes.size(); i++)
        {
            // For every circle, make an edge for each pair of nodes it has
            for (int j = 0; j < i; j++)
            {
                if (i != j)
                {
                    edge_t edge = {.n1 = circle_nodes[i], .n2 = circle_nodes[j]};
                    for (int k = 0; k < (int)circles.size(); k++)
                    {
                        blocking_edge = false;
                        // For each circle check whether it's block the hugging edges
                        if (circle_nodes[i].circle_index != k) // Not the same circle
                        {
                            // If the circles are overlap
                            if (vec_distance(circles[k].center, circles[circle_nodes[i].circle_index].center) <= (circles[k].radius + circles[circle_nodes[i].circle_index].radius))
                            {
                                blocking_edge = is_blocking_js(circles[k], circles[circle_nodes[i].circle_index]);
                            }
                        }
                        if (!blocking_edge)
                            hugging_edges.push_back(edge);
                    }
                }
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
#ifndef DEBUG_PATH
    vector<node_t> path;
    node_t current = goal_node;
    while (!(current == start_node))
    {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start_node); // optional
    reverse(path.begin(), path.end());
#else
    // for (auto edge : surfing_edges)
    //     printf("e %f %f %f %f\n", edge.n1.coord.x, edge.n1.coord.y, edge.n2.coord.x, edge.n2.coord.y);

    printf("Clear\n");

    vector<node_t> path;
    node_t current = goal_node;
    float_pair ant = current.coord;

    while (!(current == start_node))
    {
        printf("l %f %f %f %f\n", current.coord.x, current.coord.y, ant.x, ant.y);
        ant.x = current.coord.x;
        ant.y = current.coord.y;

        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start_node); // optional
    reverse(path.begin(), path.end());

    printf("l %f %f %f %f\n", current.coord.x, current.coord.y, ant.x, ant.y);
    int i = 0;
    for (auto circle : circles)
    {
        printf("c %f %f %f %i\n", circle.center.x, circle.center.y, circle.radius, i);
        i++;
    }
#endif


   objective_t obj;    
   
   if (path[0].circle_index == path[1].circle_index)
    // if we are in a hugging edge
   {
        vector<objective_t> intermediate = intermediate_steps(path[0].coord, path[1].coord, circles[path[0].circle_index]); 
        printf("FODASE KKKKKKKKKKKKKKKKKKKKKKKKKK\n");
        #ifdef DEBUG_PATH
        for(auto objective: intermediate)
        {
            printf("step %f %f\n", objective.x, objective.y);
        }
        #endif
   
        obj = intermediate[intermediate.size() - 1];
   
   }

    else // not a hugging edge
    {
        double angle;

        if (path.size() == 2) // next goal is last goal
            angle = theta;

        else
        {
            // get the angle of the surfing edge
            angle = atan2((path[1].coord.y - path[0].coord.y), (path[1].coord.x - path[0].coord.x));
            if (angle < 0)
                angle += 2* M_PI;
        }

        obj = {path[1].coord.x, path[1].coord.y, angle};
    
    }

    return obj;
}