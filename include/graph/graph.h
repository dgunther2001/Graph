#ifndef GRAPH
#define GRAPH

#include <map>
#include <vector>
#include <stack>
#include <queue>
#include <deque>
#include <set>
#include <iostream>
#include <chrono>

template<typename T>
class adj_list_entry {
    T node_name;
    int weight;

public:
    adj_list_entry(T node_name, int weight) : 
            node_name(node_name),
            weight(weight)
            {}
    
    ~adj_list_entry() {}

    int get_weight() const {
        return weight;
    }

    const T get_node() const {
        return node_name;
    }
};

template<typename T>
class graph {
private:    
    bool directed;
    std::map<T, std::vector<adj_list_entry<T>>> adj_list;

public:
    graph(bool directed) : directed(directed) {}
    ~graph() {}

    void print_nodes_debug() {
        for (auto const& [node, adj_list] : adj_list) {
            std::cout << node << "\n";
        }
    }

    void print_graph_debug() {
        for (auto const& [node, adj_list] : adj_list) {
            std::cout << node << ": ";
            for(auto const& entry : adj_list) {
                std:: cout << entry.get_node() << " ";
            }

            std::cout << "\n";
        }
    }

    void print_dfs_debug(T start_node) {
        std::vector<T> traversal = depth_first_traversal(start_node);
        std::cout << "DFS: ";
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    void print_bfs_debug(T start_node) {
        std::vector<T> traversal = breadth_first_traversal(start_node);
        std::cout << "BFS: ";
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    void print_djikstras_debug(T start_node) {
        std::vector<std::tuple<T, int, T>> djikstra_output = Djikstras(start_node);
        for (auto const& entry : djikstra_output) {
            if (std::get<1>(entry) == 65535) {
                std::cout << "Node " << std::get<0>(entry) << " is not connected to " << start_node;
                std::cout << "\n";
            }
            else {
                std::cout << "Node: " << std::get<0>(entry) << ", Distance From " << start_node << ": " << std::get<1>(entry) << ", Previous Node In Path: " << std::get<2>(entry);
                std::cout << "\n";
            }
        }
    }

    void print_djikstra_single_path_debug(T start, T end) {
        std::deque<T> path = shortest_path_djikstras(start, end);\

        std::cout << "Shortest path from " << start << " to " << end << ": "; 

        for (const auto& entry : path) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    void print_a_star_debug(T start, T end) {
        std::deque<T> a_star_output = a_star(start, end);
        for (T entry : a_star_output) {
            std:: cout << entry << " ";
        }
         std::cout << "\n";
    }

    void a_star_djikstras_time_comparison(T start, T end, int num_iterations) {
        std::vector<double> djikstras_time;
        std::vector<double> a_star_time;

        auto avg_djikstras_time = 0;
        auto avg_a_star_time = 0;

        auto start_time = std::chrono::high_resolution_clock::now();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::nano> time_taken;

        for (int i = 0; i < num_iterations; i++) {
            start_time = std::chrono::high_resolution_clock::now();
            shortest_path_djikstras(start, end);
            end_time = std::chrono::high_resolution_clock::now();
            time_taken = end_time - start_time;
            djikstras_time.emplace_back(time_taken.count());
        }

        for (double entry : djikstras_time) {
            avg_djikstras_time += entry;
        }

        avg_djikstras_time = avg_djikstras_time / num_iterations;


        for (int i = 0; i < num_iterations; i++) {
            start_time = std::chrono::high_resolution_clock::now();
            a_star(start, end);
            end_time = std::chrono::high_resolution_clock::now();
            time_taken = end_time - start_time;
            a_star_time.emplace_back(time_taken.count());
        }

        for (double entry : a_star_time) {
            avg_a_star_time += entry;
        }

        avg_a_star_time = avg_a_star_time / num_iterations;

        std::cout << "Djikstra's Average Time: " << avg_djikstras_time << " ns\n";
        std::cout << "A* Average Time: " << avg_a_star_time << " ns\n";

    }

    double heuristic_function(T start, T goal) {
       return 0; // placeholder heuristic function (BECOMES EQUIVALENT TO DJIKSTRAS)
    }

    void add_node(T node) {
        std::vector<adj_list_entry<T>> empty_adj_list;
        adj_list.insert({node, empty_adj_list});
    }
    
    void add_edge(T node_1, T node_2, int weight) {      
        if (adj_list.find(node_1) == adj_list.end()) {
            add_node(node_1);
        }

        if (adj_list.find(node_2) == adj_list.end()) {
            add_node(node_2);
        }

        adj_list_entry<T> primary_entry(node_2, weight);
        adj_list[node_1].push_back(primary_entry);

        if (!directed) {
            adj_list_entry<T> secondary_entry(node_1, weight);
            adj_list[node_2].push_back(secondary_entry);
        }
    }

    std::vector<T> depth_first_traversal(T start_node) {
        std::vector<T> traversal_list; // the returned list in order
        std::stack<T> traversal_stack; // stack used to implement traversal
        std::vector<T> visited_list; // list of nodes already visited

        std::vector<adj_list_entry<T>> current_adj_list; // stores the adjacency list of the active node
        T current_node; // stores the current node

        traversal_stack.push(start_node); // push the desired start location onto the traversal stack
        while(!traversal_stack.empty()) {

            current_node = traversal_stack.top(); // store the top of the stack
            traversal_stack.pop(); // remove it from the stack

            if (std::find(visited_list.begin(), visited_list.end(), current_node) == visited_list.end() /* visited_list.end() is returned if the entry is not found */) { // if the current node has already been visited...
                visited_list.push_back(current_node); // add the node to the visited list to avoid redundancies and incorrect results
                traversal_list.push_back(current_node); // add the current node to the traversal
                current_adj_list = adj_list[current_node]; // grab the current node's adjacency list

                for(auto const& entry : current_adj_list) { // iterate over the adjacenct list
                    if (std::find(visited_list.begin(), visited_list.end(), entry.get_node()) == visited_list.end()) { // if the node hasn't yet been visited...
                        traversal_stack.push(entry.get_node()); // add it to the stack
                    }
                }
            }

        }

        return traversal_list; // return the traversal
    }

    std::vector<T> breadth_first_traversal(T start_node) {
        std::vector<T> traversal_list; // the returned list in order
        std::queue<T> traversal_queue; // queue used to implement traversal
        std::vector<T> visited_list; // list of nodes already visited

        std::vector<adj_list_entry<T>> current_adj_list; // stores the adjacency list of the active node
        T current_node; // stores the current node

        traversal_queue.push(start_node);
        while(!traversal_queue.empty()) {
            current_node = traversal_queue.front();
            traversal_queue.pop();

            if (std::find(visited_list.begin(), visited_list.end(), current_node) == visited_list.end()) {
                visited_list.push_back(current_node); // add the node to the visited list to avoid redundancies and incorrect results
                traversal_list.push_back(current_node); // add the current node to the traversal
                current_adj_list = adj_list[current_node]; // grab the current node's adjacency list

                for(auto const& entry : current_adj_list) { // iterate over the adjacenct list
                    if (std::find(visited_list.begin(), visited_list.end(), entry.get_node()) == visited_list.end()) { // if the node hasn't yet been visited...
                        traversal_queue.push(entry.get_node()); // add it to the queue
                    }
                }
            }
        }

        return traversal_list; // return the traversal
    }

    std::vector<std::tuple<T, int, T>> Djikstras(T start) {
        std::map<T, int> distance_list;
        std::map<T, T> previous_nodes;
        std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<>> next_node_queue;
        std::vector<std::tuple<T, int, T>> path_lengths;

        for (auto const& [node, adj_vec] : adj_list) {
            distance_list[node] = 65535;
            previous_nodes[node] = T();
        }

        distance_list[start] = 0;
        next_node_queue.push({0, start});

        while (!next_node_queue.empty()) {
            std::pair<int, T> current_node = next_node_queue.top();
            next_node_queue.pop();

            for (auto const& entry : adj_list[current_node.second]) {
                int adjusted_distance = entry.get_weight() + current_node.first;
                if (adjusted_distance < distance_list[entry.get_node()]) {
                    distance_list[entry.get_node()] = adjusted_distance;
                    previous_nodes[entry.get_node()] = current_node.second;
                    next_node_queue.push({adjusted_distance, entry.get_node()});
                }
            }
        }

        for (auto const& [node, dist] : distance_list) {
            path_lengths.emplace_back(node, dist, previous_nodes[node]);
        }

        return path_lengths;
    }

    std::deque<T> shortest_path_djikstras(T start, T end) {
        std::vector<std::tuple<T, int, T>> full_djikstras = Djikstras(start);
        std::deque<T> path;

        std::tuple<T, int, T> previous_node;
        for (const auto& node : full_djikstras) {
            if (std::get<0>(node) == end) {
                previous_node = node;
                break;
            }
        }

        path.push_front(end);

        while (std::get<0>(previous_node) != start) {
            for (const auto& current_node : full_djikstras) {
                if (std::get<0>(current_node) == std::get<2>(previous_node)) {
                    path.push_front(std::get<0>(current_node));
                    previous_node = current_node;
                    break;
                }
            }
        }

        return path;

    }
    
    std::deque<T> a_star(T start, T end) {
        std::priority_queue<std::pair<double, T>, std::vector<std::pair<double, T>>, std::greater<>> to_explore; // nodes we need to explore prioritized by a lower estimated cost
        std::map<T, double> g_cost; // maps actual cost from start to end
        std::map<T, double> f_cost; // maps the estimated cost (g_cost + heurisitic)
        std::map<T, T> from; // maps each node to previous node on shorteest path
        std::set<T> closed; // fully explored nodes

        // Initialize costs
        for (const auto& [node, list] : adj_list) { // sets the g and f cost of all nodes to a value that is essentially infinity 
            g_cost[node] = 65535;
            f_cost[node] = 65535;
        }

        g_cost[start] = 0; // cost to reach itself is 0
        f_cost[start] = heuristic_function(start, end); // heuristic estimated cost from start to end
        to_explore.emplace(f_cost[start], start); // the first node is thrown into the priority queue to be explored

        while (!to_explore.empty()) {
            T current_node = to_explore.top().second; // pull the node with the lowest f cost and explore it
            to_explore.pop();

            if (current_node == end) { // if the current node we are exploring is the last one, we have found the path...
                std::deque<T> path;
                while (current_node != start) { // push nodes onto the path deque
                    path.push_front(current_node);
                    current_node = from[current_node]; // pull the prior node and repeat until we get to the first node
                }
                path.push_front(start);
                return path;
            }

            closed.insert(current_node); // mark the current node as fully explored and thus avoid redundancies

            T neighbor;
            for (const auto& neighbor_entry : adj_list[current_node]) { // iterate over all neighbor nodes of the current node being explored
                neighbor = neighbor_entry.get_node(); // set a neighbor node to explore
                double tentative_g_cost = g_cost[current_node] + neighbor_entry.get_weight(); // calculate a tentative g_cost based on the g_cost of the current node and the weight of the attachment to the neighbor

                if (closed.find(neighbor) != closed.end()) { // if the neighbor has been explored, go onto the next neighbor
                    continue;
                }

                if (tentative_g_cost < g_cost[neighbor]) { // if the new calculated g_cost is better than the current one stored, update the g_cost
                    from[neighbor] = current_node; // update the path
                    g_cost[neighbor] = tentative_g_cost; // set the new g_cost
                    f_cost[neighbor] = g_cost[neighbor] + heuristic_function(neighbor, end); // set f_cost to the current g_cost plus the value estimated by the heuristic function to the end from the neighbor (becomes more accurate over time)
                    to_explore.emplace(f_cost[neighbor], neighbor); // add the new neighbor to the priority queue to be explored
                }
            }
        }

        return {}; // if all paths are exhausted, then no path exists, so return an empty deque
    }


};

#endif