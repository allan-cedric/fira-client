#include "math_operations.h"
#include "net/robocup_ssl_client.h"
#include "header.h"

// computed: {
// nodes_and_surfing_edges: function() { return generate_nodes_and_surfing_edges(this.circles); },
// surfing_edges: function() { return this.nodes_and_surfing_edges.edges; },
// nodes: function() { return this.nodes_and_surfing_edges.nodes; },
// hugging_edges: function() { return generate_hugging_edges(this.nodes); },
// edges: function() { return this.surfing_edges.concat(this.hugging_edges); },
// path: function() { return find_path(this.circles[this.circles.length-2],
//                                     this.circles[this.circles.length-1],
//                                     this.nodes, this.edges); }




Objective path(vector<fira_message::sim_to_ref::Robot&> other_robots,
            fira_message::sim_to_ref::Robot &my_robot,
            double x, double y, double theta)
{

    vector<circle_t> circles (5);

    for(int i = 0; i < other_robots.size(); i++)
    {
        float_pair center;
        center.x = other_robots[i].x();
        center.y = other_robots[i].y();
        circle_t circle(center);
        circles.push_back(circle);
    }




}



