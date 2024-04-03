#ifndef GRAPH_HPP
#define GRAPH_HPP

// DO NOT CHANGE THIS FILE

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <map>
#include <set>
#include <vector>
#include <list>
#include "GraphNode.hpp"


template <typename T> class Graph {
private:
    /// @brief  adjacency list representation of the graph edges
    std::map<T, std::set<T>> _adjList;

    /// @brief map from vertex value to GraphNode to modify the node for BFS/DFS
    std::map<T, GraphNode<T>> _vertices;

    /// @brief helper function for DFS
    /// @param u the vertex to visit
    /// @param time the current time count
    /// @param record a list of vertices in the order of topological sort
    void DFS_visit(const T &u, int &time, std::list<T> &record);

public:
    Graph() = default;

    /// @brief construct a graph from a list of edges
    /// @param edges a vector of pairs of vertices representing the edges
    ///        each pair is two veritces <from, to>
    Graph(const std::vector<std::pair<T, T>> &edges);

    /// @brief construct a graph from an adjacency list
    /// @param adjList a map from vertex to a set of its neighbors
    Graph(const std::map<T, std::set<T>> &adjList);

    ~Graph() = default;

    /// @brief get the number of vertices in the graph
    /// @return the number of vertices in the graph
    int size() const;

    /// @brief add a vertex to the graph with no edges
    /// @param vertex the value of the vertex to add
    void addVertex(T vertex);

    /// @brief add an edge to the graph
    /// @param from the value of the vertex to add the edge from
    /// @param to the value of the vertex to add the edge to
    void addEdge(T from, T to);

    /// @brief check if an edge exists in the graph
    /// @param from the value of the vertex to check the edge from
    /// @param to the value of the vertex to check the edge to
    bool hasEdge(T from, T to) const;

    /// @brief get the neighbors of a vertex
    /// @param vertex the value of the vertex to get the neighbors of
    /// @return a set of the neighbors of the vertex if vertex is in the graph, otherwise std::nullopt
    std::optional<std::set<T>> getNeighbors(T vertex) const;


    /// @brief get the GraphNode reference for a vertex
    /// @param vertex the value of the vertex to get the GraphNode reference for
    /// @return a reference to the GraphNode for the vertex
    GraphNode<T> &operator[](const T &vertex) {
        return _vertices[vertex];
    }

    // ----------------- BFS -----------------

    /// @brief BFS traversal of the graph
    /// @param start the value of the vertex to start the traversal from
    /// @return a vector of vertices in the order of BFS traversal
    std::vector<T> BFS(T start);

    /// @brief find the length of the shortest path between two vertices
    /// @param start the starting vertex of the shortest path
    /// @param end the ending vertex of the shortest path
    /// @return the length of the shortest path between the two vertices
    int shortestPath(T start, T end);

    // ----------------- DFS -----------------

    /// @brief DFS traversal of the graph
    /// @return a list of vertices in the order of topological sort
    std::list<T> DFS();

};

#endif // GRAPH_HPP
