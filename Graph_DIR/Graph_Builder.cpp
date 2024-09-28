#include <vector>
#include <algorithm>

#ifndef GRAPH_BULDER_CPP
#define GRAPH_BULDER_CPP

/**
 * The Graph class represents an undirected graph using adjacency lists.
 * It provides functionality to add and remove edges, and to clear the graph.
 */

class Graph
{
public:
   /**
     * A structure to represent an edge in the graph.
     */
    struct Edge
    {
        int from;   ///< Starting vertex of the edge
        int to;     ///< Ending vertex of the edge
        int weight; ///< Weight of the edge
        Edge(int f, int t, int w) : from(f), to(t), weight(w) {} //Constructor for the Edge struct.
    };
    /**
     * Constructs a Graph with a specified number of vertices.
     * @param vertices Number of vertices in the graph.
     */
    Graph(int vertices) : V(vertices)
    {
        adjacencyList.resize(V);
    }
    /**
     * Default constructor for the Graph class.
     * Initializes an empty graph with no vertices.
    */
    Graph() {};

    /**
     * Adds an edge to the graph. Since the graph is undirected,
     * the edge is added in both directions.
     * 
     * @param from Starting vertex
     * @param to Ending vertex
     * @param weight Weight of the edge
     */

    void addEdge(int from, int to, int weight)
    {
        adjacencyList[from].emplace_back(from, to, weight);
        adjacencyList[to].emplace_back(to, from, weight); // For undirected graph
        this->edgecount++;
    }


    /**
     * Gets the edge between two vertices, if it exists.
     * @param from Starting vertex
     * @param to Ending vertex
     * @return The edge between the specified vertices, or a default-constructed Edge if the edge does not exist.
     */
    Graph::Edge getEdge(int from, int to) const {
        for (const auto& edge : adjacencyList[from]) {
            if (edge.to == to) {
                return edge;
            }
        }
        return {-1, -1, 0};
    }

    /**
     * Removes an edge from the graph, erases it from both vertices.
     * 
     * @param from Starting vertex
     * @param to Ending vertex
     * @param weight Weight of the edge to be removed
     * @note Dissconnecting a vertex will cause a weight of -inf around it.
     */
    void removeEdge(int from, int to, int weight)
    {
        // Find the edge to remove in the adjacency list of 'from'
        auto it = std::find_if(adjacencyList[from].begin(), adjacencyList[from].end(),
                               [&](const Edge &e)
                               { return e.to == to && e.weight == weight; });
        if (it != adjacencyList[from].end())
        {
            adjacencyList[from].erase(it);
        }

        // Find the edge to remove in the adjacency list of 'to'
        it = std::find_if(adjacencyList[to].begin(), adjacencyList[to].end(),
                          [&](const Edge &e)
                          { return e.to == from && e.weight == weight; });
        if (it != adjacencyList[to].end())
        {
            adjacencyList[to].erase(it);
        }

        this->edgecount--;
    }

    /**
     * Clears the graph, removing all edges.
     */
    void clearGraph()
    {
        for (auto &edges : adjacencyList)
        {
            edges.clear();
        }
        edgecount = 0;
    }
    /**
     * Gets the number of vertices in the graph.
     * @return The number of vertices.
     */
    int getVerticesCount() const { return V; }
    /**
     * Gets the number of edges in the graph.
     * @return The number of edges.
     */
    int getEdgesCount() const { return edgecount; }
    /**
     * Gets the adjacency list representing the graph.
     * @return A reference to the adjacency list.
     */
    const std::vector<std::vector<Edge>> &getAdjacencyList() const { return adjacencyList; }

private:
    int V; // Number of vertices
    std::vector<std::vector<Edge>> adjacencyList; //Adjacency list for storing edges
    int edgecount = 0; ///< Count of the total number of edges in the graph
};

#endif