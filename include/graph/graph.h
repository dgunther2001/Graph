#ifndef GRAPH
#define GRAPH

#include <map>
#include <vector>
#include <stack>
#include <queue>
#include <iostream>

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
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
    }

    void print_bfs_debug(T start_node) {
        std::vector<T> traversal = breadth_first_traversal(start_node);
        for (T entry : traversal) {
            std::cout << entry << " ";
        }

        std::cout << "\n";
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

};

#endif