#include <limits>
#include <iostream>

#include "Graph_Builder.cpp"

#ifndef MST_CPP
#define MST_CPP


/**
 * @brief The MST class represents a minimum spanning tree of a graph.
 *        It provides methods to analyze the MST, such as calculating
 *        the total weight, longest path, shortest edge, and average distance.
 */

class MST
{
public:
    std::vector<Graph::Edge> mstEdges;

    /**
     * Constructs an MST object using a given graph.
     * @param graph The graph from which the MST is derived.
     */

    MST(const Graph &graph) : originalGraph(graph) {}

    /**
     * Adds an edge to the MST.
     * @param from Starting vertex of the edge
     * @param to Ending vertex of the edge
     * @param weight Weight of the edge
     */

    void addEdge(int from, int to, int weight)
    {
        mstEdges.emplace_back(from, to, weight);
    }

    /**
     * Calculates the total weight of the minimum spanning tree.
     * @return The total weight of all edges in the MST.
     */
    int getTotalWeight() const
    {
        int total = 0;
        for (const auto &edge : mstEdges)
        {
            total += edge.weight;
        }
        return total;
    }
    /**
     * Finds the longest distance in the MST using depth first search (DFS). 
     * @return The longest distance between any two vertices in the MST.
     */
    int getLongestDistance() const
    {
        return dfs(0).first;
    }
    /**
     * Finds the edge with the smallest weight in the MST.
     * @return The weight of the shortest edge in the MST.
     */
    int getShortestEdge() const
    {
        if (mstEdges.empty())
            return 0;
        return std::min_element(mstEdges.begin(), mstEdges.end(),
                                [](const Graph::Edge &a, const Graph::Edge &b)
                                {
                                    return a.weight < b.weight;
                                })
            ->weight;
    }
    /**
     * Calculates the average distance between all pairs of vertices in the graph.
     * Uses the Floyd-Warshall algorithm to compute shortest paths.
     * 
     * @return The average distance between vertex pairs.
     */
    double getAverageDistance() const
    {
        int V = originalGraph.getVerticesCount();
        std::vector<std::vector<int>> dist(V, std::vector<int>(V, std::numeric_limits<int>::max()));

        // Initialize distances
        for (int i = 0; i < V; ++i)
        {
            dist[i][i] = 0;
            for (const auto &edge : originalGraph.getAdjacencyList()[i])
            {
                dist[i][edge.to] = edge.weight;
            }
        }

        // Floyd-Warshall algorithm
        for (int k = 0; k < V; ++k)
        {
            for (int i = 0; i < V; ++i)
            {
                for (int j = 0; j < V; ++j)
                {
                    if (dist[i][k] != std::numeric_limits<int>::max() &&
                        dist[k][j] != std::numeric_limits<int>::max() &&
                        dist[i][k] + dist[k][j] < dist[i][j])
                    {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        // Calculate average distance
        long long sum = 0;
        int count = 0;
        for (int i = 0; i < V; ++i)
        {
            for (int j = i + 1; j < V; ++j)
            {
                if (dist[i][j] != std::numeric_limits<int>::max())
                {
                    sum += dist[i][j];
                    count++;
                }
            }
        }
        return static_cast<double>(sum) / count;
    }

private:
    const Graph &originalGraph; ///< Reference to the original graph from which MST is derived

    /**
     * Performs a depth first search (DFS) to find the path with the maximum depth in the MST.
     * 
     * @param node Current node id in the DFS
     * @param parent Parent node id to avoid traversing back
     * 
     * @return A pair containing the maximum depth and the farthest node id reached.
     */

    std::pair<int, int> dfs(int node, int parent = -1) const
    {
        int maxDepth = 0;
        int maxNode = node;

        for (const auto &edge : mstEdges)
        {
            int nextNode = (edge.from == node) ? edge.to : (edge.to == node) ? edge.from
                                                                             : -1;
            if (nextNode != -1 && nextNode != parent)
            {
                auto [depth, farthestNode] = dfs(nextNode, node);
                depth += edge.weight;
                if (depth > maxDepth)
                {
                    maxDepth = depth;
                    maxNode = farthestNode;
                }
            }
        }

        return {maxDepth, maxNode};
    }
};

#endif
