#ifndef GRAPH
#define GRAPH

#include <map>
#include <vector>
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

};

#endif