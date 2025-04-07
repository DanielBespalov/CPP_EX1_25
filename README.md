## Graph Algorithms

This project contains implementations of various graph algorithms and data structures in C++.  
The implementation does not use the STL and is based entirely on manually managed dynamic arrays.

## Description

The project includes:

- A graph structure using adjacency lists (undirected, weighted).
- Algorithms including:
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
  - Dijkstra's algorithm for shortest paths
  - Prim's algorithm for minimum spanning tree
  - Kruskal's algorithm using Union-Find

All supporting structures such as `Queue`, `PriorityQueue`, and `UnionFind` are implemented manually.

## File Structure

- `graph.h` / `graph.cpp` - Graph class implementation
- `Algorithms.h` / `Algorithms.cpp` - Algorithms and supporting data structures
- `main.cpp` - Demonstration of graph creation and algorithm usage
- `test.cpp` - Unit tests for all functions using doctest
- `doctest.h` - Header-only testing framework
- `Makefile` - Compilation and automation file

## Graph Class

The graph is undirected and uses an adjacency list implemented with dynamic arrays.

Main methods:

- `addEdge(int from, int to, int weight = 1)`
- `removeEdge(int from, int to)`
- `print_graph()`
- `getNumVertices()`
- `getNeighbors(int vertex, int*& neighborsOut, int& sizeOut)`

Creating a graph with 0 vertices will throw an exception.

## Algorithms

The following algorithms are implemented in the `Algorithms` class:

- `Graph bfs(const Graph& g, int start)`
- `Graph dfs(const Graph& g, int start)`
- `Graph dijkstra(const Graph& g, int start)`
- `Graph prim(const Graph& g)`
- `Graph kruskal(const Graph& g)`

Each function returns a new `Graph` object representing the result (tree or forest).

## Notes

- No STL containers are used in the project.
- All dynamic memory is properly managed.
- Graph is implemented using dynamic adjacency lists.
- Algorithms assume valid inputs; invalid operations will throw exceptions.


