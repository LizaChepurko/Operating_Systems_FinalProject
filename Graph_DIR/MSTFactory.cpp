#include "Graph_Builder.cpp"
#include "MST.cpp"
#include "MSTAlgorithm.cpp"

/**
 * @brief The MSTFactory class is responsible for creating instances of 
 * different Minimum Spanning Tree (MST) algorithms. It uses the Factory 
 * design pattern to provide abstractions over various algorithm implementations.
 */
class MSTFactory
{
public:
    enum class Algorithm
    {
        BORUVKA,
        PRIM,
        KRUSKAL,
        TARJAN,
    };

    /**
     * Creates an instance of a specified MST algorithm.
     * 
     * @param algo The algorithm type selected from the Algorithm enumeration.
     * @return A unique pointer to an MSTAlgorithm object of the specified type.
     * @throws std::runtime_error if an unknown algorithm type is requested.
     */

    static std::unique_ptr<MSTAlgorithm> createAlgorithm(Algorithm algo)
    {
        switch (algo)
        {
        case Algorithm::BORUVKA:
            return std::make_unique<BoruvkaAlgorithm>();
        case Algorithm::PRIM:
            return std::make_unique<PrimAlgorithm>();
        case Algorithm::KRUSKAL:
            return std::make_unique<KruskalAlgorithm>();
        case Algorithm::TARJAN:
            return std::make_unique<TarjanAlgorithm>();
        default:
            throw std::runtime_error("Unknown algorithm");
        }
    }
};
