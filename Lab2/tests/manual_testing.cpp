#include <cstdlib>
#include <iostream>

#include <boost/graph/exception.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <boost/graph/subgraph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>

#include "../src/merge/include/merge.hpp"

class DfsVisitorImpl : public boost::default_dfs_visitor {
public:
    template <class Vertex, class Graph>
    void discover_vertex(Vertex u, Graph& g) {
        std::cout << u << ' ';
    }
};

struct VertexProperties {
    std::string name_;  // Name of the vertex
};

struct EdgeProperties {
    int weight_;  // Weight of the edge

    explicit EdgeProperties(int weight)
        : weight_(weight) {}
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                              VertexProperties, EdgeProperties>
    GraphWithProperties;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              boost::no_property,
                              boost::property<boost::edge_weight_t, int>>
    WeightedGraph;

typedef boost::subgraph<boost::adjacency_list<
    boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_index_t, int>>>
    Subgraph;

int main() {
    std::cout << "--------------TEST-CASE-1.1--------------\n";
    UndirAdjList graph_11;
    UndirAdjList::vertex_descriptor v0_graph11 = boost::add_vertex(graph_11);
    UndirAdjList::vertex_descriptor v1_graph11 = boost::add_vertex(graph_11);
    UndirAdjList::vertex_descriptor v2_graph11 = boost::add_vertex(graph_11);
    UndirAdjList::vertex_descriptor v3_graph11 = boost::add_vertex(graph_11);
    boost::add_edge(v0_graph11, v1_graph11, graph_11);
    boost::add_edge(v0_graph11, v2_graph11, graph_11);
    boost::add_edge(v1_graph11, v3_graph11, graph_11);
    boost::add_edge(v2_graph11, v3_graph11, graph_11);
    boost::remove_edge(v0_graph11, v1_graph11, graph_11);
    boost::print_graph(graph_11);

    std::cout << "--------------TEST-CASE-1.2--------------\n";
    DirAdjList graph12(4);
    boost::add_edge(0, 1, graph12);
    boost::add_edge(0, 2, graph12);
    boost::add_edge(1, 3, graph12);
    boost::add_edge(2, 3, graph12);
    boost::print_graph(graph12);

    std::cout << "--------------TEST-CASE-1.3--------------\n";
    UndirAdjMatrix graph13(4);
    boost::add_edge(0, 1, graph13);
    boost::add_edge(0, 2, graph13);
    boost::add_edge(1, 3, graph13);
    boost::add_edge(2, 3, graph13);
    boost::remove_edge(0, 1, graph13);
    boost::print_graph(graph13);

    std::cout << "--------------TEST-CASE-1.4--------------\n";
    DirAdjMatrix graph14(4);
    boost::add_edge(0, 1, graph14);
    boost::add_edge(0, 2, graph14);
    boost::add_edge(1, 3, graph14);
    boost::add_edge(2, 3, graph14);
    boost::print_graph(graph14);

    std::cout << "--------------TEST-CASE-2.1--------------\n";
    UndirAdjList graph21_1(3);
    UndirAdjList graph21_2(6);
    UndirAdjList::vertex_descriptor v3_graph21_1 = boost::add_vertex(graph21_1);
    UndirAdjList::vertex_descriptor v6_graph21_2 = boost::add_vertex(graph21_2);
    boost::add_edge(0, 1, graph21_1);
    boost::add_edge(0, 2, graph21_1);
    boost::add_edge(1, 3, graph21_1);
    boost::add_edge(2, 3, graph21_1);

    boost::add_edge(0, 1, graph21_2);
    boost::add_edge(1, 6, graph21_2);
    boost::add_edge(1, 2, graph21_2);
    boost::add_edge(2, 3, graph21_2);
    boost::add_edge(3, 6, graph21_2);
    boost::add_edge(2, 4, graph21_2);
    boost::add_edge(2, 5, graph21_2);
    mergeGraphs<UndirAdjList, UndirAdjList::vertex_descriptor>(
        graph21_1, v3_graph21_1, graph21_2, v6_graph21_2);
    boost::print_graph(graph21_1);

    std::cout << "--------------TEST-CASE-2.2--------------\n";
    UndirAdjList graph22_1(3);
    UndirAdjList graph22_2(6);
    UndirAdjList::vertex_descriptor v3_graph22_1 = boost::add_vertex(graph22_1);
    UndirAdjList::vertex_descriptor v6_graph22_2 = boost::add_vertex(graph22_2);
    boost::add_edge(0, 1, graph22_1);
    boost::add_edge(0, 2, graph22_1);
    boost::add_edge(1, 3, graph22_1);
    boost::add_edge(2, 3, graph22_1);

    boost::add_edge(0, 1, graph22_2);
    boost::add_edge(1, 6, graph22_2);
    boost::add_edge(1, 2, graph22_2);
    boost::add_edge(2, 3, graph22_2);
    boost::add_edge(3, 6, graph22_2);
    boost::add_edge(2, 4, graph22_2);
    boost::add_edge(2, 5, graph22_2);
    boost::add_edge(v3_graph22_1, v6_graph22_2, graph22_1);
    boost::print_graph(graph22_1);

    std::cout << "--------------TEST-CASE-3----------------\n";
    GraphWithProperties graph3;
    // Add vertices with properties
    GraphWithProperties::vertex_descriptor v1_graph3 =
        boost::add_vertex(graph3);
    GraphWithProperties::vertex_descriptor v2_graph3 =
        boost::add_vertex(graph3);
    GraphWithProperties::vertex_descriptor v3_graph3 =
        boost::add_vertex(graph3);
    graph3[v1_graph3].name_ = "Vertex 0";
    graph3[v2_graph3].name_ = "Vertex 1";
    graph3[v3_graph3].name_ = "Vertex 2";
    boost::add_edge(v1_graph3, v2_graph3, EdgeProperties(10), graph3);
    boost::add_edge(v2_graph3, v3_graph3, EdgeProperties(20), graph3);
    boost::add_edge(v1_graph3, v3_graph3, EdgeProperties(30), graph3);
    typename boost::property_map<GraphWithProperties,
                                 std::string VertexProperties::*>::type
        vertex_properties = get(&VertexProperties::name_, graph3);
    typename boost::graph_traits<GraphWithProperties>::vertex_iterator vi,
        vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph3); vi != vi_end; ++vi) {
        std::cout << "Name: " << vertex_properties[*vi] << '\n';
    }
    typename boost::property_map<GraphWithProperties,
                                 int EdgeProperties::*>::type edge_properties =
        get(&EdgeProperties::weight_, graph3);
    typename boost::graph_traits<GraphWithProperties>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph3); ei != ei_end; ++ei) {
        std::cout << "Weight: " << edge_properties[*ei] << '\n';
    }

    std::cout << "--------------TEST-CASE-4.1--------------\n";
    typedef boost::graph_traits<DirAdjList>::vertex_descriptor VertexDescriptor;

    const std::size_t vertex_count = 7;
    const std::size_t edge_count = 6;

    const std::array<std::pair<int, int>, edge_count> edges = {
        {{0, 1}, {0, 2}, {1, 3}, {1, 4}, {2, 5}, {2, 6}}};

    DirAdjList graph41(edges.cbegin(), edges.cend(), vertex_count);

    std::array<VertexDescriptor, vertex_count> result_order;

    // correct_order = {3, 4, 1, 5, 6, 2, 0};
    boost::topological_sort(graph41, result_order.begin());
    for (const auto& num : result_order) {
        std::cout << num << ' ';
    }
    std::cout << '\n';

    std::cout << "--------------TEST-CASE-4.2--------------\n";
    typedef boost::graph_traits<UndirAdjList>::vertex_descriptor
        VertexDescriptor;
    UndirAdjList graph42(edges.cbegin(), edges.cend(), vertex_count);
    try {
        boost::topological_sort(graph42, result_order.begin());
    } catch (const boost ::not_a_dag& e) {
        std::cout << "ERROR: " << e.what() << '\n';
    }

    std::cout << "--------------TEST-CASE-4.3--------------\n";
    UndirAdjList graph43(edges.cbegin(), edges.cend(), vertex_count);
    DfsVisitorImpl visitor;
    // correct_order = {0, 1, 3, 4, 2, 5, 6};
    boost::depth_first_search(graph43, boost::visitor(visitor).root_vertex(0u));
    std::cout << '\n';

    std::cout << "--------------TEST-CASE-4.4--------------\n";
    WeightedGraph graph44(6);
    boost::add_edge(0, 1, graph44);
    boost::add_edge(1, 2, graph44);
    boost::add_edge(1, 3, graph44);
    boost::add_edge(3, 4, graph44);
    boost::add_edge(4, 5, graph44);
    boost::add_edge(2, 3, graph44);

    boost::property_map<WeightedGraph, boost::edge_weight_t>::type weight_map =
        get(boost::edge_weight, graph44);

    boost::put(weight_map, boost::edge(0, 1, graph44).first, 2);
    boost::put(weight_map, boost::edge(1, 2, graph44).first, 4);
    boost::put(weight_map, boost::edge(1, 3, graph44).first, 6);
    boost::put(weight_map, boost::edge(2, 3, graph44).first, 1);
    boost::put(weight_map, boost::edge(3, 4, graph44).first, 3);
    boost::put(weight_map, boost::edge(4, 5, graph44).first, 2);

    std::vector<int> dist(num_vertices(graph44),
                          std::numeric_limits<int>::max());
    std::vector<WeightedGraph::vertex_descriptor> parent(num_vertices(graph44),
                                                         -1);
    dijkstra_shortest_paths(
        graph44, 0, boost::predecessor_map(&parent[0]).distance_map(&dist[0]));

    // correct_dist = {0, 2, 6, 7, 10, 12};
    for (const auto& curr_dist : dist) {
        std::cout << curr_dist << ' ';
    }
    std::cout << '\n';

    std::cout << "--------------TEST-CASE-5.1--------------\n";
    UndirAdjList graph51(11);

    boost::add_edge(0, 1, graph51);
    boost::add_edge(0, 2, graph51);
    boost::add_edge(1, 3, graph51);
    boost::add_edge(2, 3, graph51);
    boost::add_edge(4, 5, graph51);
    boost::add_edge(3, 10, graph51);
    boost::add_edge(5, 10, graph51);
    boost::add_edge(5, 6, graph51);
    boost::add_edge(7, 10, graph51);
    boost::add_edge(6, 7, graph51);
    boost::add_edge(6, 8, graph51);
    boost::add_edge(6, 9, graph51);

    boost::remove_edge(6, 7, graph51);
    boost::remove_edge(7, 10, graph51);

    boost::remove_vertex(7, graph51);
    boost::print_graph(graph51);

    std::cout << "--------------TEST-CASE-5.2--------------\n";
    UndirAdjList graph52(11);

    boost::add_edge(0, 1, graph52);
    boost::add_edge(0, 2, graph52);
    boost::add_edge(1, 3, graph52);
    boost::add_edge(2, 3, graph52);
    boost::add_edge(4, 5, graph52);
    boost::add_edge(3, 10, graph52);
    boost::add_edge(5, 10, graph52);
    boost::add_edge(5, 6, graph52);
    boost::add_edge(7, 10, graph52);
    boost::add_edge(6, 7, graph52);
    boost::add_edge(6, 8, graph52);
    boost::add_edge(6, 9, graph52);

    boost::remove_vertex(10, graph52);
    boost::print_graph(graph52);

    std::cout << "--------------TEST-CASE-6----------------\n";
    Subgraph graph6(6);

    boost::add_edge(0, 1, graph6);
    boost::add_edge(1, 2, graph6);
    boost::add_edge(1, 3, graph6);
    boost::add_edge(1, 4, graph6);
    boost::add_edge(3, 5, graph6);
    boost::add_edge(4, 5, graph6);

    Subgraph& subgraph1 = graph6.create_subgraph();
    Subgraph& subgraph2 = graph6.create_subgraph();

    boost::add_vertex(0, subgraph1);
    boost::add_vertex(1, subgraph1);

    boost::add_vertex(2, subgraph2);
    boost::add_vertex(4, subgraph2);
    boost::add_vertex(5, subgraph2);

    std::cout << "Subgraph1: \n";
    boost::print_graph(subgraph1);
    std::cout << "Subgraph2: \n";
    boost::print_graph(subgraph2);

    return EXIT_SUCCESS;
}