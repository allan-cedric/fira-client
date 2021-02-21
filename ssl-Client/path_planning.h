#ifndef PATH_PLANNING
#define PATH_PLANNING

#include "math_operations.h"
#include "net/robocup_ssl_client.h"
#include "header.h"
#include <map>

template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

void add_edge(vector<edge_t> &surfing_edges, vector<circle_t> circles, vector<node_t> &nodes, int i,
              float_pair P, int j, float_pair Q);

node_t circle_to_node(int circle_index, vector<node_t> nodes);

bool node_comparison(node_t N, node_t M);

vector<node_t> neighbors(node_t node, vector<edge_t> edges);

double edge_cost(node_t a, node_t b, vector<circle_t> circles);

double heuristic(node_t node);

Objective path(vector<fira_message::sim_to_ref::Robot> &other_robots, fira_message::sim_to_ref::Robot &my_robot, 
               double x, double y, double theta);

#endif