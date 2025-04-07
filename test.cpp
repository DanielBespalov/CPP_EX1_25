#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "graph.h"
#include "algorithms.h"

#include <limits.h>

using namespace graph;

TEST_CASE("Graph basic operations") {
    Graph g(5);
    CHECK(g.getNumVertices() == 5);

    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 3);

    int* neighbors = nullptr;
    int size = 0;

    g.getNeighbors(0, neighbors, size);
    CHECK(size == 2);

    CHECK_THROWS(g.removeEdge(0, 4)); // לא קיימת

    g.removeEdge(0, 1);
    g.getNeighbors(0, neighbors, size);
    CHECK(size == 1);
}

TEST_CASE("BFS builds a tree") {
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    Graph tree = Algorithms::bfs(g, 0);
    int totalEdges = 0;

    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 == 3); // עץ על 4 קודקודים = 3 צלעות
}

TEST_CASE("DFS builds a tree") {
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(2, 3);

    Graph tree = Algorithms::dfs(g, 0);
    int totalEdges = 0;

    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 == 3);
}

TEST_CASE("Dijkstra builds shortest path tree") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(1, 2, 2);
    g.addEdge(0, 2, 10);
    g.addEdge(2, 3, 1);

    Graph tree = Algorithms::dijkstra(g, 0);

    int totalEdges = 0;
    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 == 3);
}

TEST_CASE("Prim builds MST") {
    Graph g(5);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 3, 6);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 8);
    g.addEdge(1, 4, 5);
    g.addEdge(2, 4, 7);
    g.addEdge(3, 4, 9);

    Graph tree = Algorithms::prim(g);
    int totalEdges = 0;

    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 == 4); // MST על 5 קודקודים = 4 צלעות
}

TEST_CASE("Kruskal builds MST") {
    Graph g(5);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 3, 6);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 8);
    g.addEdge(1, 4, 5);
    g.addEdge(2, 4, 7);
    g.addEdge(3, 4, 9);

    Graph tree = Algorithms::kruskal(g);
    int totalEdges = 0;

    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 == 4);
}

TEST_CASE("Empty graph throws") {
    CHECK_THROWS_AS(Graph(0), std::invalid_argument);
}

TEST_CASE("Invalid vertex access is handled") {
    Graph g(3);
    int* neighbors = nullptr;
    int size = -1;

    g.getNeighbors(-1, neighbors, size);
    CHECK(size == 0);

    g.getNeighbors(5, neighbors, size);
    CHECK(size == 0);

    CHECK_THROWS(g.removeEdge(0, 10));
}

TEST_CASE("Disconnected graph BFS/DFS returns partial tree") {
    Graph g(6);
    g.addEdge(0, 1);
    g.addEdge(2, 3); // רכיב אחר

    Graph bfs_tree = Algorithms::bfs(g, 0);
    Graph dfs_tree = Algorithms::dfs(g, 0);

    int total_bfs = 0, total_dfs = 0;
    int* neighbors = nullptr;
    int size = 0;

    for (int i = 0; i < g.getNumVertices(); ++i) {
        bfs_tree.getNeighbors(i, neighbors, size);
        total_bfs += size;

        dfs_tree.getNeighbors(i, neighbors, size);
        total_dfs += size;
    }

    CHECK(total_bfs / 2 < 5); // לא כל הגרף מחובר
    CHECK(total_dfs / 2 < 5);
}

TEST_CASE("Self loop does not crash or duplicate") {
    Graph g(3);
    g.addEdge(0, 0);
    g.addEdge(1, 1, 2);

    int* neighbors = nullptr;
    int size = 0;

    g.getNeighbors(0, neighbors, size);
    CHECK(size == 1);

    g.getNeighbors(1, neighbors, size);
    CHECK(size == 1);

    CHECK_NOTHROW(g.removeEdge(1, 1));
    CHECK_THROWS(g.removeEdge(1, 1)); // כבר הוסרה
}

TEST_CASE("Negative weights with Kruskal/Prim") {
    Graph g(4);
    g.addEdge(0, 1, -2);
    g.addEdge(1, 2, -1);
    g.addEdge(2, 3, -3);

    Graph mst1 = Algorithms::kruskal(g);
    Graph mst2 = Algorithms::prim(g);

    int edges1 = 0, edges2 = 0;
    int* neighbors = nullptr;
    int size = 0;

    for (int i = 0; i < g.getNumVertices(); ++i) {
        mst1.getNeighbors(i, neighbors, size);
        edges1 += size;

        mst2.getNeighbors(i, neighbors, size);
        edges2 += size;
    }

    CHECK(edges1 / 2 == 3);
    CHECK(edges2 / 2 == 3);
}

TEST_CASE("Adding the same edge twice does not duplicate it") {
    Graph g(3);
    g.addEdge(0, 1, 5);
    g.addEdge(0, 1, 5); // should not be duplicated

    int* neighbors = nullptr;
    int size = 0;
    g.getNeighbors(0, neighbors, size);
    CHECK(size == 1);
}

TEST_CASE("Self-loop is handled correctly") {
    Graph g(2);
    g.addEdge(0, 0, 7);

    int* neighbors = nullptr;
    int size = 0;
    g.getNeighbors(0, neighbors, size);
    CHECK(size == 1);
    CHECK(neighbors[0] == 0);
    CHECK(neighbors[1] == 7);
}

TEST_CASE("Invalid vertex in BFS and DFS throws") {
    Graph g(4);
    CHECK_THROWS(Algorithms::bfs(g, -1));
    CHECK_THROWS(Algorithms::dfs(g, 10));
}

TEST_CASE("Disconnected graph in BFS does not visit all nodes") {
    Graph g(6);
    g.addEdge(0, 1);
    g.addEdge(2, 3);

    Graph tree = Algorithms::bfs(g, 0);
    int totalEdges = 0;
    for (int i = 0; i < tree.getNumVertices(); ++i) {
        int* neighbors = nullptr;
        int size = 0;
        tree.getNeighbors(i, neighbors, size);
        totalEdges += size;
    }

    CHECK(totalEdges / 2 < 5); // should be less than 5 edges in the BFS tree
}

TEST_CASE("Dijkstra on disconnected graph does not connect all vertices") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    g.addEdge(2, 3, 1); // disconnected component

    Graph tree = Algorithms::dijkstra(g, 0);
    int* neighbors = nullptr;
    int size = 0;
    tree.getNeighbors(2, neighbors, size);
    CHECK(size == 0); // unreachable from 0
}
