#ifndef GRAPH_NODE_HPP
#define GRAPH_NODE_HPP

// DO NOT CHANGE THIS FILE
#include <optional>

enum Color { White, Gray, Black };

template<typename T>
class GraphNode {
public:
    T value;
    Color color;

    // for BFS
    int distance;
    std::optional<T> predecessor;

    // for DFS
    int discovery_time;
    int finish_time;

    GraphNode() = default;

    // a Node is only constructed with a value
    // all other fields are initialized to default values
    GraphNode(T value) :
        value(value),
        color(White),
        distance(-1),
        predecessor(std::nullopt),
        discovery_time(-1),
        finish_time(-1)
    {}

    // copy constructor
    GraphNode(const GraphNode<T> &other) :
        value(other.value),
        color(other.color),
        distance(other.distance),
        predecessor(other.predecessor),
        discovery_time(other.discovery_time),
        finish_time(other.finish_time)
    {}
};

#endif // GRAPH_NODE_HPP

