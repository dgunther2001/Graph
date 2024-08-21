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

    // implementing adj matrix as well...
    std::map<T, int> matrix_index_map;
    std::vector<std::vector<int>> adjacency_matrix;

public:
    graph(bool directed) : directed(directed) {}
    ~graph() {}

    const void print_nodes_debug() {
        for (auto const& [node, adj_list] : adj_list) {
            std::cout << node << "\n";
        }
    }

    const void print_graph_debug() {
        for (auto const& [node, adj_list] : adj_list) {
            std::cout << node << ": ";
            for(auto const& entry : adj_list) {
                std:: cout << entry.get_node() << " ";
            }

            std::cout << "\n";
        }
    }

    const void print_adj_matrix_debug() {
        int count = 0;
        std::cout << "\t";
        for (int i = 0; i < adjacency_matrix.size(); i++) {
            for (auto const& [key, index_mapping] : matrix_index_map) {
                if (index_mapping == count) {
                    std::cout << "{" << key << "}" << "\t";
                }
            }
            count++;
        }

        std::cout << "\n";

        count = 0;
        for (std::vector<int> adj_vector : adjacency_matrix) { 

            for (auto const& [key, index_mapping] : matrix_index_map) {
                if (index_mapping == count) {
                    std::cout << "{" << key << "}" << "\t";
                }
                
            }
            
            count++;

            std::cout << " ";
            for (int weight_val : adj_vector) {
                std::cout << weight_val << "\t";
                std::cout << " ";
            }
            std::cout << "\n";
        }
    }

    const void print_dfs_debug(T start_node) {
        std::vector<T> traversal = depth_first_traversal(start_node);
        std::cout << "DFS: ";
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    const void print_bfs_debug(T start_node) {
        std::vector<T> traversal = breadth_first_traversal(start_node);
        std::cout << "BFS: ";
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    const void print_djikstras_debug(T start_node) {
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

    const void print_djikstra_single_path_debug(T start, T end) {
        std::deque<T> path = shortest_path_djikstras(start, end);\

        std::cout << "Shortest path from " << start << " to " << end << ": "; 

        for (const auto& entry : path) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    const void print_a_star_debug(T start, T end) {
        std::deque<T> a_star_output = a_star(start, end);
        for (T entry : a_star_output) {
            std:: cout << entry << " ";
        }
         std::cout << "\n";
    }

    const void print_topological_sort_debug() {
        std::vector<T> topo_sort = topological_sort();
        for (T entry : topo_sort) {
            std::cout << entry << " ";
        }
        std::cout << "\n";
    }

    const void a_star_djikstras_time_comparison(T start, T end, int num_iterations) {
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

    const double heuristic_function(T start, T goal) {
       return 0; // placeholder heuristic function (BECOMES EQUIVALENT TO DJIKSTRAS)
    }

     void add_node(T node) {
        // ADD ERROR HANDLING FOR IF KEY EXISTS

        // hash implementation
        std::vector<adj_list_entry<T>> empty_adj_list;
        adj_list.insert({node, empty_adj_list});

        // adj matrix implementation
        int new_index = adjacency_matrix.size();
        matrix_index_map.insert({node, new_index});
        
        for (std::vector<int>& adjacent_vec : adjacency_matrix) { // apppend 0s to the entire adjacency matrix
            adjacent_vec.push_back(0);
        }

        std::vector<int> new_vector;
        for (int i = 0; i <= new_index; i++) {
            new_vector.push_back(0);
        }

        adjacency_matrix.push_back(new_vector);

    }
    
    void add_edge(T node_1, T node_2, int weight) {      
        if (adj_list.find(node_1) == adj_list.end()) {
            add_node(node_1);
        }

        if (adj_list.find(node_2) == adj_list.end()) {
            add_node(node_2);
        }

        // hash implementation
        adj_list_entry<T> primary_entry(node_2, weight);
        adj_list[node_1].push_back(primary_entry);

        if (!directed) {
            adj_list_entry<T> secondary_entry(node_1, weight);
            adj_list[node_2].push_back(secondary_entry);
        }

        // adjacency matrix implementation
        int node_1_index = matrix_index_map[node_1];
        int node_2_index = matrix_index_map[node_2];
        adjacency_matrix[node_1_index][node_2_index] = weight;

        if (!directed) {
            adjacency_matrix[node_2_index][node_1_index] = weight;
        }
    }

    const std::vector<T> depth_first_traversal(T start_node) {
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

    const std::vector<T> breadth_first_traversal(T start_node) {
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

    const std::vector<std::tuple<T, int, T>> Djikstras(T start) {
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

   const std::deque<T> shortest_path_djikstras(T start, T end) {
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
    
    const std::deque<T> a_star(T start, T end) {
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

    const bool is_acyclic() {
        if (directed == false) {
            return false;
        }

        // iterate over the nodes in the graph and fully explore them looking for cycles
        // how to do this for specific nodes
            // do a depth first traversal on each and every node and look for cycles (see if it connects back to itself)
            // if it does, return false, otherwise after we have executed a dfs for each node and not detected any cycles, return true
    
        // what i need...
        // a call stack that stores the current DFS that resets with the for loop
        std::stack<T> current_traversal;
        // a set of visited nodes that stores the current path of the call stack that removes nodes when specific paths have been resolved
        std::set<T> visited_in_current_iteration;

        for (auto const& [node, list] : adj_list) { // iterates over each node in the graph
            
            current_traversal.push(node); // push the node onto the stack and proceed with a standard DFS

            T current_node; // declare a current_node variable for the DFS

            while(!current_traversal.empty()) { // while the DFS can proceed...
                current_node = current_traversal.top(); // pull the current node off of the top of the stack and store it
                current_traversal.pop(); // pop the stack

                for (auto const& entry : adj_list[current_node]) { // iterate over the adjacency list of the current node
                    if (visited_in_current_iteration.find(entry.get_node()) != visited_in_current_iteration.end()) { // if the node has previously been visited in the current DFS, return false as this indicates a back edge
                        return false;
                    }
                    current_traversal.push(entry.get_node());// otherwise put the node on the stack
                    visited_in_current_iteration.emplace(entry.get_node()); // add the node to the set of visited nodes for the current DFS iteration
                }

                visited_in_current_iteration.erase(current_node); // once a particular node has been fully explored, remove it as we are no longer exploring that path
            }

        }
        
        return true;

    }

    const std::vector<T> topological_sort() {
        if (!is_acyclic()) {
            throw std::runtime_error("Graph is not acyclic, so topological sort cannot be done.");
        }

        // what i need to do here 
        // overall, iterate over nodes and remove ones without incoming edges and add them to my topological sort
        // keep removing dependencies until all nodes have been explored (trickle up)

        // first => find nodes without dependencies =>  NO INCOMIGN EDGES
            // have a std::map<T, int> that stores map<node, incoming edges> that includes all of the nodes in the graph
                // must initialize this by first adding every node into the map and initializing the incoming edges to 0
                // then iterate over each adjacency list and increment the respective value of outgoing edges in the map

        std::map<T, int> incoming_edges;
        std::vector<T> topological_ordering;

        for (auto const& [node, adj] : adj_list) {
            incoming_edges.insert({node, 0});
        }

        for (auto const& [node, adj] : adj_list) {
            for (auto const& entry : adj) {
                incoming_edges[entry.get_node()] += 1;
            }
        }

        // now we need to initialize a queue of places to explore with no indegree
        std::deque<T> no_in_deg;

        for (auto const& [node, incoming] : incoming_edges) {
            if (incoming == 0) {
                no_in_deg.push_front(node);
            }
        }

        T current_node;
        while(!no_in_deg.empty()) {
            // remove item with no indegree from consideration and add it to the topological sort REMOVE IT FROM THE incoming edges map
            // in addition, we must then adjust all indegree of all nodes that have that node as an in degree

            current_node = no_in_deg.back(); // pull a node off of the queue with nodes without an in degree
            no_in_deg.pop_back(); // pull it off the queue
            topological_ordering.push_back(current_node); // push the current node onto the topological ordering

            incoming_edges.erase(current_node); // remove current_node from the map as it is in the topological ordering now
            
            for (auto const& entry : adj_list[current_node]) {
                incoming_edges[entry.get_node()]--;
            }

            for (auto const& [node, incoming] : incoming_edges) {
                if (incoming == 0 && (std::find(no_in_deg.begin(), no_in_deg.end(), node) == no_in_deg.end())) {
                    no_in_deg.push_front(node);
                }
            }

        }

        return topological_ordering;

    }



};

#endif