#include <iostream>
#include "graph/graph.h"

int main() {
    std::cout << "Graph Tests\n";

    graph<char> my_graph(false);
    
    my_graph.add_node('A');
    my_graph.add_node('B');
    my_graph.add_node('C');
    my_graph.add_node('D');
    my_graph.add_node('E');
    my_graph.add_node('F');

    my_graph.add_edge('B', 'A', 7);
    my_graph.add_edge('C', 'F', 16);
    my_graph.add_edge('D', 'A', 9);
    my_graph.add_edge('B', 'E', 31);
    my_graph.add_edge('C', 'D', 3);
    my_graph.add_edge('A', 'C', 1);

    //my_graph.print_dfs_debug(1);

    //my_graph.print_bfs_debug(1);

    //my_graph.print_djikstras_debug('A');

    my_graph.print_djikstra_single_path_debug('A', 'F');

    return 0;
}