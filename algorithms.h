#pragma once
#include "graph.h"

namespace graph {

// ==== Queue ====
class Queue {
private:
    int* data;
    int front, rear, size, capacity;
public:
    Queue(int cap);
    ~Queue();
    void enqueue(int value);
    int dequeue();
    bool isEmpty() const;
};

// ==== PriorityQueue ====
class PriorityQueue {
private:
    int* data;
    int* priority;
    bool* inQueue;
    int size, capacity;
public:
    PriorityQueue(int cap);
    ~PriorityQueue();
    void insert(int value, int prio);
    void decreasePriority(int value, int newPrio);
    int extractMin();
    bool isEmpty() const;
};

// ==== UnionFind ====
class UnionFind {
private:
    int* parent;
    int* rank;
    int size;
public:
    UnionFind(int n);
    ~UnionFind();
    int find(int x);
    void unite(int x, int y);
};

// ==== Algorithms ====
class Algorithms {
public:
    static Graph bfs(const Graph& g, int start);
    static Graph dfs(const Graph& g, int start);
    static Graph dijkstra(const Graph& g, int start);
    static Graph prim(const Graph& g);
    static Graph kruskal(const Graph& g);
};

} // namespace graph
