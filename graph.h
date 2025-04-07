#pragma once
/**
 * danieldaniel2468@gmail.com
 */

namespace graph {

class Graph {
private:
    int numVertices;
    int** neighbors;
    int* sizes;
    int* capacities;

    void resize(int vertex);
    void addNeighbor(int from, int to, int weight);
    bool removeNeighbor(int from, int to);

public:
    Graph(int vertices);
    ~Graph();

    void addEdge(int from, int to, int weight = 1);
    void removeEdge(int from, int to);
    void print_graph();

    int getNumVertices() const;
    void getNeighbors(int vertex, int*& neighborsOut, int& sizeOut) const;
};

}