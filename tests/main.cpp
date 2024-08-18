#include <iostream>
#include "graph/graph.h"

int main() {
    std::cout << "Graph Tests\n";

    graph<int> my_graph(true);
    my_graph.add_node(1);
    //my_graph.print_nodes_debug();

    std::cout << "\n";

    my_graph.add_node(17);
    //my_graph.print_nodes_debug();

    my_graph.add_edge(1, 2, 17);

    my_graph.print_graph_debug();

    return 0;
}