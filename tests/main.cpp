#include <iostream>
#include "graph/graph.h"

int main() {
    std::cout << "Graph Tests\n";

    graph<int> my_graph(false);
    
    my_graph.add_node(1);
    my_graph.add_node(65);
    my_graph.add_node(31);
    my_graph.add_node(44);
    my_graph.add_node(2);
    my_graph.add_node(77);

    my_graph.add_edge(65, 1, 1);
    my_graph.add_edge(1, 31, 1);
    my_graph.add_edge(31, 2, 1);
    my_graph.add_edge(2, 44, 1);
    my_graph.add_edge(77, 1, 1);
    my_graph.add_edge(65, 44, 1);

    my_graph.print_dfs_debug(1);

    my_graph.print_bfs_debug(1);

    return 0;
}