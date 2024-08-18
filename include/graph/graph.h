#ifndef GRAPH
#define GRAPH

#include <map>
#include <vector>
#include <iostream>

template<typename T>
class vector_entry {
    T node_name;
    int weight;
};

template<typename T>
class graph {
private:    
    bool directed;
    std::map<T, std::vector<vector_entry<T>>> adj_list;

public:
    graph(bool directed) : directed(directed) {}
    ~graph() {}

    void print_nodes_debug() {
        for (auto const& [node, adj_list] : adj_list) {
            std::cout << node << "\n";
        }
    }

    void add_node(T node) {
        std::vector<vector_entry<T>> empty_adj_list;
        adj_list.insert({node, empty_adj_list});
    }

};

#endif