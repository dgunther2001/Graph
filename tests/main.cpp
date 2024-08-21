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
    my_graph.add_node('G');

    my_graph.add_edge('B', 'A', 7);
    my_graph.add_edge('C', 'F', 16);
    my_graph.add_edge('D', 'A', 9);
    my_graph.add_edge('B', 'E', 31);
    my_graph.add_edge('C', 'D', 3);
    my_graph.add_edge('A', 'C', 1);

    my_graph.print_adj_matrix_debug();

    //my_graph.print_dfs_debug(1);

    //my_graph.print_bfs_debug(1);

    //my_graph.print_djikstras_debug('A');

    //my_graph.print_djikstra_single_path_debug('A', 'F');

    //my_graph.print_djikstras_debug('A');

    //std::cout << my_graph.heuristic_function('A', 'E') << "\n";

    //my_graph.a_star_djikstras_time_comparison('A', 'F', 10000);

    //std::cout << my_graph.is_acyclic() << "\n";

    /*
    graph<char> my_graph2(true);
    my_graph2.add_node('A');
    my_graph2.add_node('C');
    my_graph2.add_node('G');
    my_graph2.add_node('F');

    my_graph2.add_edge('A', 'C', 1);
    my_graph2.add_edge('C', 'G', 1);
    my_graph2.add_edge('C', 'F', 1);

    //std::cout << my_graph2.is_acyclic() << "\n";

    //my_graph2.add_edge('F', 'A', 1);

    //std::cout << my_graph2.is_acyclic() << "\n";

    my_graph2.topological_sort('A');

    */

    graph<int> my_graph3(true);

    my_graph3.add_node(1);
    my_graph3.add_node(7);
    my_graph3.add_node(33);
    my_graph3.add_node(77);
    my_graph3.add_node(3);
    my_graph3.add_node(6);

    my_graph3.add_edge(1, 7, 1);
    my_graph3.add_edge(7, 33, 1);
    my_graph3.add_edge(33, 6, 1);
    my_graph3.add_edge(7, 77, 1);
    my_graph3.add_edge(77, 6, 1);
    my_graph3.add_edge(1, 77, 1);
    my_graph3.add_edge(3, 77, 1);

    //std::cout << my_graph3.is_acyclic() << "\n";

    //my_graph3.print_topological_sort_debug();

    //my_graph3.print_adj_matrix_debug();


    //my_graph3.topological_sort(1);

    return 0;
}