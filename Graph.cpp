#ifndef GRAPH_CPP
#define GRAPH_CPP

#include "Graph.hpp"
#include "GraphNode.hpp"
#include <queue>

// TODO: Implement all other methods defined in Graph.hpp here

/// @brief helper function for DFS
/// @param u the vertex to visit
/// @param time the current time count
/// @param record a list of vertices in the order of topological sort
template <typename T>
void Graph<T>::DFS_visit(const T &u, int &time, std::list<T> &record)
{
    time++;
    _vertices[u].discovery_time = time;
    _vertices[u].color = Gray;

    for(auto& vertex : _adjList[u])
    {
        if (_vertices[vertex].color == White)
        {
            _vertices[vertex].predecessor = u;
            DFS_visit(vertex, time, record);
        }
    }
    time++;
    _vertices[u].finish_time = time;
    _vertices[u].color = Black;
    record.push_front(u);
}

/// @brief construct a graph from a list of edges
/// @param edges a vector of pairs of vertices representing the edges
///        each pair is two veritces <from, to>
template <typename T>
Graph<T>::Graph(const std::vector<std::pair<T, T>> &edges)
{
    for (const std::pair<T, T> &edge : edges)
    {
        addEdge(edge.first, edge.second);
    }
}

/// @brief construct a graph from an adjacency list
/// @param adjList a map from vertex to a set of its neighbors
template <typename T>
Graph<T>::Graph(const std::map<T, std::set<T>> &adjList)
{
    for (auto &edge : adjList)
    {
        addVertex(edge.first);
    }
    _adjList = adjList;
}

/// @brief get the number of vertices in the graph
/// @return the number of vertices in the graph
template <typename T>
int Graph<T>::size() const
{
    return _vertices.size();
}

/// @brief add a vertex to the graph with no edges
/// @param vertex the value of the vertex to add
template <typename T>
void Graph<T>::addVertex(T vertex)
{
    GraphNode<T> newNode(vertex);
    _vertices[vertex] = newNode;
    _adjList[vertex];
}

/// @brief add an edge to the graph
/// @param from the value of the vertex to add the edge from
/// @param to the value of the vertex to add the edge to
template <typename T>
void Graph<T>::addEdge(T from, T to)
{
    addVertex(from);
    addVertex(to);
    _adjList[from].insert(to);
}

/// @brief check if an edge exists in the graph
/// @param from the value of the vertex to check the edge from
/// @param to the value of the vertex to check the edge to
template <typename T>
bool Graph<T>::hasEdge(T from, T to) const
{
    for (auto &edge : _adjList)
    {
        if (edge.first == from)
        {
            if (edge.second.find(to) != edge.second.end())
            {
                return true;
            }
        }
    }
    return false;
}

/// @brief get the neighbors of a vertex
/// @param vertex the value of the vertex to get the neighbors of
/// @return a set of the neighbors of the vertex if vertex is in the graph, otherwise std::nullopt
template <typename T>
std::optional<std::set<T>> Graph<T>::getNeighbors(T vertex) const
{
    for (const auto &edge : _adjList)
    {
        if (edge.first == vertex)
        {
            return edge.second;
        }
    }
    return std::nullopt;
}

// ----------------- BFS -----------------

/// @brief BFS traversal of the graph
/// @param start the value of the vertex to start the traversal from
/// @return a vector of vertices in the order of BFS traversal
template <typename T>
std::vector<T> Graph<T>::BFS(T start)
{
    std::vector<T> result;

    if (_vertices.find(start) == _vertices.end())
    {
        return result;
    }

    for(auto& vertex : _vertices)
    {
        vertex.second.color = White;
        vertex.second.predecessor = std::nullopt;
        vertex.second.distance = 0;
    }

    _vertices[start].color = Gray;
    _vertices[start].distance = 0;

    std::queue<T> q;
    q.push(start);

    while (!q.empty())
    {
        T u = q.front();
        q.pop();

        for (auto &vertex : _adjList[u])
        {
            if (_vertices[vertex].color == White)
            {
                _vertices[vertex].color = Gray;
                _vertices[vertex].distance = _vertices[u].distance + 1;
                _vertices[vertex].predecessor = u;
                q.push(vertex);
                
            }
        }
        _vertices[u].color = Black;
        result.push_back(u);
    }

    return result;
}

/// @brief find the length of the shortest path between two vertices
/// @param start the starting vertex of the shortest path
/// @param end the ending vertex of the shortest path
/// @return the length of the shortest path between the two vertices
template <typename T>
int Graph<T>::shortestPath(T start, T end)
{
    std::vector<T> list = BFS(start);

    if(list.size() == 0 || _vertices.find(end) == _vertices.end() || start == end || _vertices.find(start) == _vertices.end())
    {
        return 0;
    }

    

    return _vertices[end].distance;
    
    
    
}

// ----------------- DFS -----------------

/// @brief DFS traversal of the graph
/// @return a list of vertices in the order of topological sort
template <typename T>
std::list<T> Graph<T>::DFS()
{
    std::list<T> result;
    int time = 0;

    for(auto& vertex : _vertices)
    {
        if(vertex.second.color == White)
        {
            DFS_visit(vertex.first, time, result);
        }
    }

    return result;
}

#endif // GRAPH_CPP