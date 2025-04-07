#include "algorithms.h"
#include <climits>
#include <stdexcept>

namespace graph {

// ===== Queue =====
Queue::Queue(int cap) : front(0), rear(0), size(0), capacity(cap) {
    data = new int[capacity];
}
Queue::~Queue() { delete[] data; }
void Queue::enqueue(int value) {
    if (size == capacity) return;
    data[rear] = value;
    rear = (rear + 1) % capacity;
    size++;
}
int Queue::dequeue() {
    if (isEmpty()) return -1;
    int val = data[front];
    front = (front + 1) % capacity;
    size--;
    return val;
}
bool Queue::isEmpty() const { return size == 0; }

// ===== PriorityQueue =====
PriorityQueue::PriorityQueue(int cap) : size(0), capacity(cap) {
    data = new int[capacity];
    priority = new int[capacity];
    inQueue = new bool[capacity];
    for (int i = 0; i < capacity; ++i) {
        priority[i] = INT_MAX;
        inQueue[i] = false;
    }
}
PriorityQueue::~PriorityQueue() {
    delete[] data;
    delete[] priority;
    delete[] inQueue;
}
void PriorityQueue::insert(int value, int prio) {
    if (value < 0 || value >= capacity) return;
    data[size++] = value;
    priority[value] = prio;
    inQueue[value] = true;
}
void PriorityQueue::decreasePriority(int value, int newPrio) {
    if (value < 0 || value >= capacity) return;
    if (!inQueue[value]) return;
    if (newPrio < priority[value]) {
        priority[value] = newPrio;
    }
}
int PriorityQueue::extractMin() {
    int minIdx = -1;
    for (int i = 0; i < size; ++i) {
        if (inQueue[data[i]]) {
            if (minIdx == -1 || priority[data[i]] < priority[data[minIdx]]) {
                minIdx = i;
            }
        }
    }
    if (minIdx == -1) return -1;
    int minVal = data[minIdx];
    inQueue[minVal] = false;
    data[minIdx] = data[--size];
    return minVal;
}
bool PriorityQueue::isEmpty() const {
    for (int i = 0; i < size; ++i) {
        if (inQueue[data[i]]) return false;
    }
    return true;
}

// ===== UnionFind =====
UnionFind::UnionFind(int n) : size(n) {
    parent = new int[n];
    rank = new int[n];
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
        rank[i] = 0;
    }
}
UnionFind::~UnionFind() {
    delete[] parent;
    delete[] rank;
}
int UnionFind::find(int x) {
    if (parent[x] != x) parent[x] = find(parent[x]);
    return parent[x];
}
void UnionFind::unite(int x, int y) {
    int rootX = find(x);
    int rootY = find(y);
    if (rootX == rootY) return;
    if (rank[rootX] < rank[rootY]) {
        parent[rootX] = rootY;
    } else if (rank[rootX] > rank[rootY]) {
        parent[rootY] = rootX;
    } else {
        parent[rootY] = rootX;
        rank[rootX]++;
    }
}

/**
 * Computes the shortest paths from a source vertex to all other vertices using Dijkstra's algorithm.
 * Returns a shortest-path tree rooted at the source.
 *
 * @param g     The input weighted undirected graph.
 * @param start The source vertex index.
 * @return      A graph representing the shortest-path tree.
 * @throws      std::invalid_argument If the start index is out of bounds.
 */

Graph Algorithms::dijkstra(const Graph& g, int start) {
    int n = g.getNumVertices();
    if (start < 0 || start >= n)
        throw std::invalid_argument("Invalid start vertex");

    Graph tree(n);
    int* dist = new int[n];
    bool* visited = new bool[n]();
    int* prev = new int[n];

    for (int i = 0; i < n; ++i) {
        dist[i] = INT_MAX;
        prev[i] = -1;
    }
    dist[start] = 0;

    PriorityQueue pq(n);
    for (int i = 0; i < n; ++i)
        pq.insert(i, dist[i]);

    while (!pq.isEmpty()) {
        int u = pq.extractMin();
        if (u == -1 || dist[u] == INT_MAX) break;

        visited[u] = true;

        int* neighbors = nullptr;
        int size = 0;
        g.getNeighbors(u, neighbors, size);

        for (int i = 0; i < size; ++i) {
            int v = neighbors[i * 2];
            int w = neighbors[i * 2 + 1];
            if (!visited[v] && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.decreasePriority(v, dist[v]);
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        if (prev[i] != -1) {
            int* neighbors = nullptr;
            int size = 0;
            g.getNeighbors(i, neighbors, size);
            for (int j = 0; j < size; ++j) {
                if (neighbors[j * 2] == prev[i]) {
                    tree.addEdge(i, prev[i], neighbors[j * 2 + 1]);
                    break;
                }
            }
        }
    }

    delete[] dist;
    delete[] visited;
    delete[] prev;
    return tree;
}


/**
 * Performs a Breadth-First Search (BFS) starting from a given vertex.
 * Constructs and returns a BFS tree rooted at the source vertex.
 * The resulting tree contains only tree edges (no cross/back edges).
 * 
 * @param g     The input undirected graph.
 * @param start The source vertex to start BFS from.
 * @return      A new graph representing the BFS tree.
 * @throws      std::invalid_argument if start is an invalid vertex index.
 */


Graph Algorithms::bfs(const Graph& g, int start) {
    int n = g.getNumVertices();
    if (start < 0 || start >= n)
        throw std::invalid_argument("Invalid start vertex");

    Graph tree(n);
    bool* visited = new bool[n]();
    Queue q(n);
    visited[start] = true;
    q.enqueue(start);

    while (!q.isEmpty()) {
        int u = q.dequeue();
        int* neighbors = nullptr;
        int size = 0;
        g.getNeighbors(u, neighbors, size);

        for (int i = 0; i < size; ++i) {
            int v = neighbors[i * 2];
            int w = neighbors[i * 2 + 1];
            if (!visited[v]) {
                visited[v] = true;
                tree.addEdge(u, v, w);
                q.enqueue(v);
            }
        }
    }

    delete[] visited;
    return tree;
}

// ===== DFS =====
void dfsUtil(const Graph& g, int u, bool* visited, Graph& tree) {
    visited[u] = true;
    int* neighbors = nullptr;
    int size = 0;
    g.getNeighbors(u, neighbors, size);

    for (int i = 0; i < size; ++i) {
        int v = neighbors[i * 2];
        int w = neighbors[i * 2 + 1];
        if (!visited[v]) {
            tree.addEdge(u, v, w);
            dfsUtil(g, v, visited, tree);
        }
    }
}

/**
 * Performs a Depth-First Search (DFS) starting from a given vertex.
 * Constructs a DFS tree or forest rooted at the source, containing only tree edges.
 *
 * @param g     The input undirected graph.
 * @param start The source vertex index.
 * @return      A graph representing the DFS tree or forest.
 * @throws      std::invalid_argument If the start index is out of bounds.
 */

Graph Algorithms::dfs(const Graph& g, int start) {
    int n = g.getNumVertices();
    if (start < 0 || start >= n)
        throw std::invalid_argument("Invalid start vertex");

    Graph tree(n);
    bool* visited = new bool[n]();
    dfsUtil(g, start, visited, tree);
    delete[] visited;
    return tree;
}

/**
 * Computes a Minimum Spanning Tree (MST) using Prim's algorithm.
 * Returns a new graph containing the MST edges.
 *
 * @param g The input weighted undirected graph.
 * @return  A graph representing the MST.
 */

Graph Algorithms::prim(const Graph& g) {
    int n = g.getNumVertices();
    Graph tree(n);
    bool* inMST = new bool[n]();
    int* key = new int[n];
    int* parent = new int[n];

    for (int i = 0; i < n; ++i) {
        key[i] = INT_MAX;
        parent[i] = -1;
    }
    key[0] = 0;

    PriorityQueue pq(n);
    for (int i = 0; i < n; ++i)
        pq.insert(i, key[i]);

    while (!pq.isEmpty()) {
        int u = pq.extractMin();
        if (u == -1) break;
        inMST[u] = true;

        int* neighbors = nullptr;
        int size = 0;
        g.getNeighbors(u, neighbors, size);

        for (int i = 0; i < size; ++i) {
            int v = neighbors[i * 2];
            int w = neighbors[i * 2 + 1];
            if (!inMST[v] && w < key[v]) {
                key[v] = w;
                parent[v] = u;
                pq.decreasePriority(v, key[v]);
            }
        }
    }

    for (int v = 1; v < n; ++v) {
        if (parent[v] != -1) {
            int* neighbors = nullptr;
            int size = 0;
            g.getNeighbors(v, neighbors, size);
            for (int j = 0; j < size; ++j) {
                if (neighbors[j * 2] == parent[v]) {
                    tree.addEdge(v, parent[v], neighbors[j * 2 + 1]);
                    break;
                }
            }
        }
    }

    delete[] inMST;
    delete[] key;
    delete[] parent;
    return tree;
}

/**
 * Computes a Minimum Spanning Tree (MST) using Kruskal's algorithm.
 * Returns a new graph containing the MST edges.
 *
 * @param g The input weighted undirected graph.
 * @return  A graph representing the MST.
 */

Graph Algorithms::kruskal(const Graph& g) {
    int n = g.getNumVertices();
    Graph tree(n);
    UnionFind uf(n);

    // נאסוף את כל הצלעות
    struct Edge {
        int u, v, w;
    };
    Edge* edges = new Edge[n * n];
    int edgeCount = 0;

    for (int u = 0; u < n; ++u) {
        int* neighbors = nullptr;
        int size = 0;
        g.getNeighbors(u, neighbors, size);
        for (int i = 0; i < size; ++i) {
            int v = neighbors[i * 2];
            int w = neighbors[i * 2 + 1];
            if (u < v) {
                edges[edgeCount++] = {u, v, w};
            }
        }
    }

    // מיון הצלעות לפי משקל (מיון פשוט)
    for (int i = 0; i < edgeCount - 1; ++i) {
        for (int j = i + 1; j < edgeCount; ++j) {
            if (edges[j].w < edges[i].w) {
                Edge temp = edges[i];
                edges[i] = edges[j];
                edges[j] = temp;
            }
        }
    }

    for (int i = 0; i < edgeCount; ++i) {
        int u = edges[i].u;
        int v = edges[i].v;
        int w = edges[i].w;
        if (uf.find(u) != uf.find(v)) {
            uf.unite(u, v);
            tree.addEdge(u, v, w);
        }
    }

    delete[] edges;
    return tree;
}

} 