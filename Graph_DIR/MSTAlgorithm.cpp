#include <queue>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "Graph_Builder.cpp"
#include "MST.cpp"

#ifndef MSTAlgorithm_CPP
#define MSTAlgorithm_CPP

/**
 * @brief Abstract class for MST Algorithm implementations.
 *        Defines the interface to find MST.
 */

class MSTAlgorithm
{
public:
    /**
     * Pure virtual function to find the MST of the provided graph.
     * @param graph Reference to the graph object.
     * @return A unique pointer to an MST object representing the minimum spanning tree.
     */
    virtual std::unique_ptr<MST> findMST(const Graph &graph) = 0;
    virtual ~MSTAlgorithm() = default;
};

// Disjoint Set data structure to support union-find operations
// commonly used in Kruskal's and Borůvka's algorithms.
class DisjointSet
{
private:
    std::vector<int> parent, rank;

public:
    /**
     * Initializes a Disjoint Set with a specified number of elements.
     * @param n Number of elements.
     */
    DisjointSet(int n) : parent(n), rank(n, 0)
    {
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }
    /**
     * Finds the set representative of the given element.
     * @param x Element to find the set representative.
     * @return The root representative of the set containing x.
     */
    int find(int x)
    {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }
    /**
     * Unites two sets containing elements x and y.
     * @param x First element.
     * @param y Second element.
     */
    void unite(int x, int y)
    {
        int xroot = find(x), yroot = find(y);
        if (xroot == yroot)
            return;
        if (rank[xroot] < rank[yroot])
            parent[xroot] = yroot;
        else if (rank[xroot] > rank[yroot])
            parent[yroot] = xroot;
        else
        {
            parent[yroot] = xroot;
            rank[xroot]++;
        }
    }
};

/**
 * Borůvka's Algorithm for finding the MST of a graph.
 */
class BoruvkaAlgorithm : public MSTAlgorithm
{
public:
    /**
     * Finds the MST using Borůvka's algorithm.
     * @param graph Reference to the graph object.
     * @return A unique pointer to an MST object representing the minimum spanning tree.
     */
    std::unique_ptr<MST> findMST(const Graph &graph) override
    {
        auto mst = std::make_unique<MST>(graph);
        int V = graph.getVerticesCount();
        DisjointSet ds(V);

        // Store the cheapest edge for each component
        std::vector<Graph::Edge> cheapest(V, {-1, -1, std::numeric_limits<int>::max()});

        int numComponents = V;

        while (numComponents > 1)
        {
            // Find the cheapest edge for each component
            for (int i = 0; i < V; ++i)
            {
                for (const auto &edge : graph.getAdjacencyList()[i])
                {
                    int set1 = ds.find(edge.from);
                    int set2 = ds.find(edge.to);

                    if (set1 != set2)
                    {
                        if (edge.weight < cheapest[set1].weight)
                        {
                            cheapest[set1] = edge;
                        }
                        if (edge.weight < cheapest[set2].weight)
                        {
                            cheapest[set2] = edge;
                        }
                    }
                }
            }

            // Add the cheapest edges to the MST
            for (int i = 0; i < V; ++i)
            {
                if (cheapest[i].from != -1)
                {
                    int set1 = ds.find(cheapest[i].from);
                    int set2 = ds.find(cheapest[i].to);

                    if (set1 != set2)
                    {
                        mst->addEdge(cheapest[i].from, cheapest[i].to, cheapest[i].weight);
                        ds.unite(set1, set2);
                        numComponents--;
                    }
                }
            }

            // Reset cheapest edges for next iteration
            std::fill(cheapest.begin(), cheapest.end(), Graph::Edge(-1, -1, std::numeric_limits<int>::max()));
        }

        return mst;
    }
};

/**
 *  Prim's Algorithm for finding the MST of a graph.
 */
class PrimAlgorithm : public MSTAlgorithm
{
public:
    /**
     * Finds the MST using Prim's algorithm.
     * @param graph Reference to the graph object.
     * @return A unique pointer to an MST object representing the minimum spanning tree.
     */
    std::unique_ptr<MST> findMST(const Graph &graph) override
    {
        auto mst = std::make_unique<MST>(graph);
        int V = graph.getVerticesCount();
        std::vector<int> key(V, std::numeric_limits<int>::max());
        std::vector<bool> inMST(V, false);
        std::vector<int> parent(V, -1);

        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

        key[0] = 0;
        pq.push({0, 0});

        while (!pq.empty())
        {
            int u = pq.top().second;
            pq.pop();

            if (inMST[u])
                continue;

            inMST[u] = true;

            for (const auto &edge : graph.getAdjacencyList()[u])
            {
                int v = edge.to;
                int weight = edge.weight;

                if (!inMST[v] && key[v] > weight)
                {
                    key[v] = weight;
                    pq.push({key[v], v});
                    parent[v] = u;
                }
            }
        }

        for (int i = 1; i < V; ++i)
        {
            mst->addEdge(parent[i], i, key[i]);
        }

        return mst;
    }
};

/**
 * Kruskal's Algorithm for finding the MST of a graph.
 */
class KruskalAlgorithm : public MSTAlgorithm
{
public:
    /**
     * Finds the MST using Kruskal's algorithm.
     * @param graph Reference to the graph object.
     * @return A unique pointer to an MST object representing the minimum spanning tree.
     */
    std::unique_ptr<MST> findMST(const Graph &graph) override
    {
        auto mst = std::make_unique<MST>(graph);
        int V = graph.getVerticesCount();
        std::vector<Graph::Edge> edges;

        for (int i = 0; i < V; ++i)
        {
            for (const auto &edge : graph.getAdjacencyList()[i])
            {
                if (edge.from < edge.to)
                { // To avoid duplicates in undirected graph
                    edges.push_back(edge);
                }
            }
        }

        std::sort(edges.begin(), edges.end(), [](const Graph::Edge &a, const Graph::Edge &b)
                  { return a.weight < b.weight; });

        DisjointSet ds(V);

        for (const auto &edge : edges)
        {
            int set1 = ds.find(edge.from);
            int set2 = ds.find(edge.to);

            if (set1 != set2)
            {
                mst->addEdge(edge.from, edge.to, edge.weight);
                ds.unite(set1, set2);
            }
        }

        return mst;
    }
};

/**
 * A modified version of Tarjan's Algorithm for finding the MST of a graph.
 */
class TarjanAlgorithm : public MSTAlgorithm
{
public:
    /**
     * Finds the MST using a variant of Tarjan's algorithm.
     * @param graph Reference to the graph object.
     * @return A unique pointer to an MST object representing the minimum spanning tree.
     */
    std::unique_ptr<MST> findMST(const Graph &graph) override
    {
        int V = graph.getVerticesCount();
        std::vector<Graph::Edge> edges = gatherEdges(graph);

        // Sort all edges in ascending order
        std::sort(edges.begin(), edges.end(), [](const Graph::Edge &a, const Graph::Edge &b)
                  { return a.weight < b.weight; });

        DisjointSet ds(V);
        auto mst = std::make_unique<MST>(graph);

        for (const auto &edge : edges)
        {
            int u = ds.find(edge.from);
            int v = ds.find(edge.to);

            if (u != v)
            {
                mst->addEdge(edge.from, edge.to, edge.weight); // Use original edge vertices
                ds.unite(u, v);
            }
        }

        return mst;
    }

private:
    /**
     * Gathers all edges from the graph for processing.
     * @param graph Reference to the graph object.
     * @return A vector containing all edges of the graph.
     */
    std::vector<Graph::Edge> gatherEdges(const Graph &graph)
    {
        std::vector<Graph::Edge> edges;
        const auto &adj = graph.getAdjacencyList();

        for (size_t i = 0; i < adj.size(); ++i)
        {
            for (const auto &edge : adj[i])
            {
                if (edge.from < edge.to)
                {
                    edges.push_back(edge);
                }
            }
        }

        return edges;
    }
};

#endif