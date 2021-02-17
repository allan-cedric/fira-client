#include "path_planning.h"

// nodes_and_surfing_edges: function() { return generate_nodes_and_surfing_edges(this.circles); },
// surfing_edges: function() { return this.nodes_and_surfing_edges.edges; },
// nodes: function() { return this.nodes_and_surfing_edges.nodes; },
// hugging_edges: function() { return generate_hugging_edges(this.nodes); },
// edges: function() { return this.surfing_edges.concat(this.hugging_edges); },
// path: function() { return find_path(this.circles[this.circles.length-2],
//                                     this.circles[this.circles.length-1],
//                                     this.nodes, this.edges); }

// try to add edge from circle i point P to circle j point Q
void add_edge(vector<edge_t> &surfing_edges, vector<circle_t> circles, vector<node_t> &nodes, int i,
              float_pair P, int j, float_pair Q)
{
    if (!line_of_sight(circles, i, P, j, Q))
        return;

    // (i, P) (j, Q)
    node_t n1 = {.coord = {.x = P.x, .y = P.y}, .circle_index = i};
    nodes.push_back(n1); // we need to store them nodes

    node_t n2 = {.coord = {.x = Q.x, .y = Q.y}, .circle_index = j};
    nodes.push_back(n2);

    edge_t edge = {.n1 = n1, .n2 = n2};

    surfing_edges.push_back(edge); // and we need to store how 2 nodes connect with each other
}

node_t circle_to_node(int circle_index, vector<node_t> nodes)
{
    vector<node_t> nodes_on_circle;

    for (auto node : nodes)
    {
        if (node.circle_index == circle_index)
            nodes_on_circle.push_back(node);
    }

    if (nodes_on_circle.size() != 1)
        printf("start/goal should be on r=0 circle\n");

    return nodes_on_circle[0];
}

bool node_comparison(node_t N, node_t M)
{
    if (N.circle_index == M.circle_index)
    {
        if (N.coord.x == M.coord.x && N.coord.y == M.coord.y)
            return true;
    }

    return false;
}

vector<node_t> neighbors(node_t node, vector<edge_t> edges)
{
    vector<node_t> results;

    for (auto edge : edges)
    {
        if (node_comparison(edge.n1, node))
            results.push_back(edge.n2);
        if (node_comparison(edge.n2, node))
            results.push_back(edge.n1);
    }
    return results;
}

double edge_cost(node_t a, node_t b, vector<circle_t> circles)
{
    // adding 1 to each edge cost to favor fewer nodes in the path
    if (a.circle_index == b.circle_index)
    // if these nodes are in the same circle
    {
        // hugging edge

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

double heuristic(node_t node)
{
    return 0; // TODO: not working yet
    //return vec_distance(goal_node, node);
}

Objective path(vector<fira_message::sim_to_ref::Robot &> other_robots, fira_message::sim_to_ref::Robot &my_robot, 
               double x, double y, double theta)
{

    // transforming robots into circles
    vector<circle_t> circles(5);

    for (int i = 0; i < other_robots.size(); i++)
    {
        float_pair center;
        center.x = other_robots[i].x();
        center.y = other_robots[i].y();
        circle_t circle(center);
        circles.push_back(circle);
    }

    // implementation details:
    // for special reasons we are not going to tell you,
    // we need the goal and the current position to be circles as well.
    // these circles are special, though: their radius is equal to 0.

    // start circle
    float_pair center;
    center.x = my_robot.x();
    center.y = my_robot.y();

    circle_t start_circle(center);
    start_circle.radius = 0;

    // goal circle
    center.x = x;
    center.y = y;

    circle_t goal_circle(center);
    goal_circle.radius = 0;

    circles.push_back(start_circle);
    circles.push_back(goal_circle);

    // getting the bitangents for each pair of circles
    // we are also adding every "surfing edge" (straight lines) between two nodes
    // to the vector of edges, and storing the nodes in the vector of nodes.

    vector<edge_t> surfing_edges;
    vector<node_t> nodes;

    for (int i = 0; i < circles.size(); i++)
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

    // generating hugging_edges (circle arcs)
    vector<vector<node_t>> nodes_in_each_circle(circles.size());

    // get, for each circle, all the nodes it has
    for (auto node : nodes)
        nodes_in_each_circle[node.circle_index].push_back(node);

    vector<edge_t> hugging_edges;

    for (auto circle_nodes : nodes_in_each_circle)
    {
        for (int i = 0; i < circle_nodes.size(); i++)
        {
            // for every circle, make an edge for each pair of nodes it has
            for (int j = 0; j < i; j++)
            {
                edge_t edge = {.n1 = circle_nodes[i], .n2 = circle_nodes[j]};
                hugging_edges.push_back(edge);
            }
        }
    }

    // now, storing all the edges together:
    surfing_edges.insert(surfing_edges.end(), hugging_edges.begin(), hugging_edges.end());

    // FINDING THE PATH, FINALLY?
    node_t start_node = circle_to_node(circles.size() - 2, nodes);
    node_t goal_node = circle_to_node(circles.size() - 1, nodes);

    /*let frontier = [[start_node, 0]];
    let came_from = new Map([[start_node, null]]);
    let cost_so_far = new Map([[start_node, 0]]);

    while (frontier.length > 0)
    {
        frontier.sort((a, b) = > a[1] - b[1]);
        let current = frontier.shift()[0];
        if (current == = goal_node)
        {
            break;
        }
        for (let next of neighbors(current))
        {
            let new_cost = cost_so_far.get(current) + edge_cost(current, next);
            if (!cost_so_far.has(next) || new_cost < cost_so_far.get(next))
            {
                cost_so_far.set(next, new_cost);
                came_from.set(next, current);
                frontier.push([ next, new_cost + heuristic(next), vec_distance(goal_node, next) ]);
            }
        }
    }*/
}
