#ifndef LAB3_SRC_GRAPH_VALIDATION
#define LAB3_SRC_GRAPH_VALIDATION

#include <array>
#include <vector>

#include <fstream>

#include <boost/graph/subgraph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>

struct VertexProperties {
    std::string name_;
};

struct EdgeProperties {
    int weight_;

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

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              boost::no_property,
                              boost::property<boost::edge_weight_t, int>>
    WeightedAdjList;

typedef boost::adjacency_matrix<boost::undirectedS, boost::no_property,
                                boost::property<boost::edge_weight_t, int>>
    WeightedAdjMatrix;

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                              VertexProperties, EdgeProperties>
    GraphWithProperties;

typedef boost::graph_traits<DirAdjList>::vertex_descriptor DirAdjListVertexDesc;

typedef boost::graph_traits<DirAdjMatrix>::vertex_descriptor
    DirAdjMatrixVertexDesc;

template <typename Graph>
[[nodiscard]] bool isValidGraphStructure(
    const Graph& graph, const std::string& correct_output_file_path);

[[nodiscard]] bool isValidAlgAns(const std::vector<int>& answer_array,
                                 const std::string& correct_output_file_path);

template <uint64_t N>
[[nodiscard]] bool isValidTopSort(
    const std::array<DirAdjListVertexDesc, N>& answer_array,
    const std::string& correct_output_file_path);

#endif  // LAB3_SRC_GRAPH_VALIDATION
