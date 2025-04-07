/**
 * danieldaniel2468@gmail.com
 */

#include "graph.h"
#include <iostream>
#include <stdexcept>

namespace graph {

    /**
     * Constructor for the Graph class.
     * Initializes an undirected graph with a fixed number of vertices.
     * Each vertex has an expandable dynamic array of neighbors and weights.
     *
     * @param vertices Number of vertices in the graph.
     * @throws std::invalid_argument if the number of vertices is <= 0.
     */

    Graph::Graph(int vertices) : numVertices(vertices) {
        if (vertices <= 0)
            throw std::invalid_argument("Graph must have at least one vertex");

        neighbors = new int*[numVertices];
        sizes = new int[numVertices];
        capacities = new int[numVertices];

        for (int i = 0; i < numVertices; ++i) {
            capacities[i] = 2; // Initial capacity
            sizes[i] = 0;
            neighbors[i] = new int[capacities[i] * 2]; // each neighbor = [vertex, weight]
        }
    }

    /**
     * Destructor to free all dynamically allocated memory.
     */

    Graph::~Graph() {
        for (int i = 0; i < numVertices; ++i) {
            delete[] neighbors[i];
        }
        delete[] neighbors;
        delete[] sizes;
        delete[] capacities;
    }

    /**
     * Doubles the capacity of the adjacency list for a given vertex.
     *
     * @param v The vertex whose adjacency list needs resizing.
     */

    void Graph::resize(int v) {
        int newCap = capacities[v] * 2;
        int* newArr = new int[newCap * 2];
        for (int i = 0; i < sizes[v] * 2; ++i) {
            newArr[i] = neighbors[v][i];
        }
        delete[] neighbors[v];
        neighbors[v] = newArr;
        capacities[v] = newCap;
    }

    /**
     * Adds a neighbor to a given vertex.
     * Prevents duplicate entries.
     *
     * @param from   The source vertex.
     * @param to     The destination vertex.
     * @param weight The weight of the edge.
     */

    void Graph::addNeighbor(int from, int to, int weight) {
        for (int i = 0; i < sizes[from]; ++i) {
            if (neighbors[from][i * 2] == to) return; // already exists
        }

        if (sizes[from] >= capacities[from]) resize(from);

        neighbors[from][sizes[from] * 2] = to;
        neighbors[from][sizes[from] * 2 + 1] = weight;
        sizes[from]++;
    }

    /**
     * Removes a neighbor from a given vertex.
     *
     * @param from The source vertex.
     * @param to   The neighbor to remove.
     * @return     True if removed, false if not found.
     */

    bool Graph::removeNeighbor(int from, int to) {
        if (from < 0 || from >= numVertices) return false;

        for (int i = 0; i < sizes[from]; ++i) {
            if (neighbors[from][i * 2] == to) {
                sizes[from]--;
                neighbors[from][i * 2] = neighbors[from][sizes[from] * 2];
                neighbors[from][i * 2 + 1] = neighbors[from][sizes[from] * 2 + 1];
                return true;
            }
        }
        return false;
    }

    /**
     * Adds an undirected edge between two vertices with a given weight.
     *
     * @param from   The source vertex.
     * @param to     The destination vertex.
     * @param weight The weight of the edge (default is 1).
     */

    void Graph::addEdge(int from, int to, int weight) {
        if (from < 0 || to < 0 || from >= numVertices || to >= numVertices) return;
        addNeighbor(from, to, weight);
        addNeighbor(to, from, weight);
    }

    /**
     * Removes an undirected edge between two vertices.
     *
     * @param from Source vertex.
     * @param to   Destination vertex.
     * @throws std::runtime_error if vertices are invalid or edge doesn't exist.
     */

    void Graph::removeEdge(int from, int to) {
        if (from < 0 || to < 0 || from >= numVertices || to >= numVertices)
            throw std::runtime_error("Invalid vertex index");

        bool removed = removeNeighbor(from, to);
        if (from != to) {
            removed = removeNeighbor(to, from) && removed;
        }
        if (!removed) {
            throw std::runtime_error("Edge does not exist");
        }
    }

    /**
     * Prints the entire graph as adjacency lists.
     * Displays all vertices and their neighbors with weights.
     */

    void Graph::print_graph() {
        for (int i = 0; i < numVertices; ++i) {
            std::cout << "Vertex " << i << ": ";
            for (int j = 0; j < sizes[i]; ++j) {
                std::cout << "(" << neighbors[i][j * 2] << ", w=" << neighbors[i][j * 2 + 1] << ") ";
            }
            std::cout << "\n";
        }
    }

    /**
     * Returns the number of vertices in the graph.
     *
     * @return Number of vertices.
     */

    int Graph::getNumVertices() const {
        return numVertices;
    }

    /**
     * Provides access to the neighbors of a given vertex.
     *
     * @param vertex       The vertex to retrieve neighbors from.
     * @param neighborsOut Pointer to array of neighbors and weights [v1, w1, v2, w2, ...].
     * @param sizeOut      Number of neighbors (not number of elements).
     */

    void Graph::getNeighbors(int vertex, int*& neighborsOut, int& sizeOut) const {
        if (vertex < 0 || vertex >= numVertices) {
            neighborsOut = nullptr;
            sizeOut = 0;
            return;
        }
        sizeOut = sizes[vertex];
        neighborsOut = neighbors[vertex];
    }

} 
