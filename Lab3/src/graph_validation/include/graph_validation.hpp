#ifndef LAB3_SRC_GRAPH_VALIDATION
#define LAB3_SRC_GRAPH_VALIDATION

#include <array>
#include <vector>
#include <cstdint>

#include <boost/graph/subgraph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>

struct VertexProperties {
    std::string name_;  // Name of the vertex
};

struct EdgeProperties {
    int weight_;  // Weight of the edge

    explicit EdgeProperties(int weight)
        : weight_(weight) {}
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>
    UndirAdjList;

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS>
    DirAdjList;

typedef boost::adjacency_matrix<boost::undirectedS> UndirAdjMatrix;

typedef boost::adjacency_matrix<boost::directedS> DirAdjMatrix;

typedef boost::subgraph<boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_index_t, int>>>
    Subgraph;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              boost::no_property,
                              boost::property<boost::edge_weight_t, int>>
    WeightedGraph;

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                              VertexProperties, EdgeProperties>
    GraphWithProperties;

template <typename Graph, uint64_t N>
[[nodiscard]] bool isValidStructure(
    const Graph& graph, const std::array<std::vector<int>, N>& neighbours);

template <typename Graph, typename VertexProperties, typename Property>
[[nodiscard]] bool isValidVertexProperty(
    Graph& graph, std::vector<Property>& correct_vertices_property);

template <typename Graph, typename EdgeProperties, typename Property>
[[nodiscard]] bool isValidEdgeProperty(
    Graph& graph, std::vector<Property>& correct_edges_property);

#endif  // LAB3_SRC_GRAPH_VALIDATION
