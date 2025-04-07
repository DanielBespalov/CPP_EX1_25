#include <iostream>
#include "graph.h"
#include "algorithms.h"

using namespace graph;

int main() {
    std::cout << "Creating graph with 5 vertices\n";
    Graph g(5);

    std::cout << "Adding edges...\n";
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 1);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 5);
    g.addEdge(2, 3, 8);
    g.addEdge(3, 4, 3);

    std::cout << "\nOriginal graph:\n";
    g.print_graph();

    std::cout << "\nRemoving edge (1,3)...\n";
    g.removeEdge(1, 3);
    g.print_graph();

    std::cout << "\nRunning BFS from vertex 0...\n";
    Graph bfsTree = Algorithms::bfs(g, 0);
    bfsTree.print_graph();

    std::cout << "\nRunning DFS from vertex 0...\n";
    Graph dfsTree = Algorithms::dfs(g, 0);
    dfsTree.print_graph();

    std::cout << "\nRunning Dijkstra from vertex 0...\n";
    Graph dijkstraTree = Algorithms::dijkstra(g, 0);
    dijkstraTree.print_graph();

    std::cout << "\nComputing MST using Prim...\n";
    Graph primTree = Algorithms::prim(g);
    primTree.print_graph();

    std::cout << "\nComputing MST using Kruskal...\n";
    Graph kruskalTree = Algorithms::kruskal(g);
    kruskalTree.print_graph();

    return 0;
}
