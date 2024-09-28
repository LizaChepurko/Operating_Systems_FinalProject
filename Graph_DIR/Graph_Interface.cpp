#include <sstream>

#include "Graph_Builder.cpp"
#include "MST.cpp"
#include "MSTAlgorithm.cpp"
#include "MSTFactory.cpp"

/**
 * Main function to interactively manage graphs and solve MST problems for them.
 * Strategy design pattern.
 * 
 * This function continuously reads commands from standard input for managing graphs,
 * like adding/removing edges or graphs, and solving the MST using specified algorithms.
 * Communicates with server process.
 * 
 * @return int The exit status of the program.
 */

int main()
{
    std::unordered_map<int, Graph> graphs;
    std::string line;
    while (std::getline(std::cin, line)) 
    {
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd; //Extract command from the line

        if (cmd == "ADD_GRAPH")
        {
            int graphId, numVertices;
            iss >> graphId >> numVertices;
            Graph graph(numVertices);
            int from, to, weight;
            while (iss >> from >> to >> weight)
            {
                graph.addEdge(from, to, weight);
            }
            graphs[graphId] = graph;
            std::cout << "Graph added successfully" << std::endl
                      << std::endl;
        }
        else if (cmd == "ADD_EDGE")
        {
            int graphId, from, to, weight;
            iss >> graphId >> from >> to >> weight;

            if (graphs.find(graphId) == graphs.end())
            {
                std::cout << "Error: Graph not found" << std::endl
                          << std::endl;
                continue;
            }

            graphs[graphId].addEdge(from, to, weight);
            std::cout << "Edge added successfully" << std::endl
                      << std::endl;
        }
        else if (cmd == "REMOVE_EDGE")
        {
            int graphId, from, to, weight;
            iss >> graphId >> from >> to >> weight;

            if (graphs.find(graphId) == graphs.end())
            {
                std::cout << "Error: Graph not found" << std::endl
                          << std::endl;
                continue;
            }

            graphs[graphId].removeEdge(from, to, weight);
            std::cout << "Edge removed successfully" << std::endl
                      << std::endl;
        }
        else if (cmd == "REMOVE_GRAPH")
        {
            int graphId;
            iss >> graphId;

            if (graphs.find(graphId) == graphs.end())
            {
                std::cout << "Error: Graph not found" << std::endl
                          << std::endl;
                continue;
            }
            // Remove the graph by clearing it. The graph object remains to avoid 
            // invalid graph IDs in the future.
            graphs[graphId].clearGraph();
            std::cout << "Graph removed successfully" << std::endl
                      << std::endl;
        }
        else if (cmd == "SOLVE_MST")
        {
            int graphId;
            std::string algoName;
            iss >> graphId >> algoName;

            if (graphs.find(graphId) == graphs.end())
            {
                std::cout << "Error: Graph not found" << std::endl
                          << std::endl;
                continue;
            }
            // Determine which algorithm to use based on user input
            MSTFactory::Algorithm algo;
            if (algoName == "PRIM")
                algo = MSTFactory::Algorithm::PRIM;
            else if (algoName == "KRUSKAL")
                algo = MSTFactory::Algorithm::KRUSKAL;
            else if (algoName == "BORUVKA")
                algo = MSTFactory::Algorithm::BORUVKA;
            else if (algoName == "TARJAN")
                algo = MSTFactory::Algorithm::TARJAN;
            else
            {
                std::cout << "Error: Unknown algorithm" << std::endl
                          << std::endl;
                continue;
            }
            // Solve the MST for the selected graph and algorithm
            auto mstAlgorithm = MSTFactory::createAlgorithm(algo);
            auto mst = mstAlgorithm->findMST(graphs[graphId]);
            // Output the results of the MST
            int totalWeight = mst->getTotalWeight();
            int longestDistance = mst->getLongestDistance();
            double avgDistance = mst->getAverageDistance();
            int shortestDistance = mst->getShortestEdge();

            std::cout << "Graph " << graphId << " MST Results:" << std::endl
                      << "Total Weight: " << totalWeight << std::endl
                      << "Longest Distance of MST: " << longestDistance << std::endl
                      << "Average Distance between two vertices of the graph: " << avgDistance << std::endl
                      << "Shortest Distance of MST: " << shortestDistance << "\n\n"
                      << std::endl;
        }
        fflush(stdin);
    }
    return 0;
}
