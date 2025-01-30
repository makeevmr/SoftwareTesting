#include <gtest/gtest.h>

#include <boost/graph/exception.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "../src/graph_validation/include/graph_validation.hpp"
#include "../src/merge/include/merge.hpp"

class DfsVisitorImpl : public boost::default_dfs_visitor {
public:
    explicit DfsVisitorImpl(const uint64_t& vertex_count)
        : boost::default_dfs_visitor() {
        visit_order.reserve(vertex_count);
    }

    template <class Vertex, class Graph>
    void discover_vertex(Vertex u, Graph& g) {
        visit_order.push_back(u);
    }

    static std::vector<uint64_t> visit_order;
};

std::vector<uint64_t> DfsVisitorImpl::visit_order;

// Verifying the creation of the described undirected adjacency list
TEST(GraphCreation, UndirAdjList) {
    constexpr uint64_t kN = 4;
    UndirAdjList undir_graph;

    std::array<std::vector<int>, kN> undir_graph_neighbours = {
        std::vector<int>{2},
        std::vector<int>{3},
        std::vector<int>{0, 3},
        std::vector<int>{1, 2},
    };

    // Add vertices
    UndirAdjList::vertex_descriptor v0 = boost::add_vertex(undir_graph);
    UndirAdjList::vertex_descriptor v1 = boost::add_vertex(undir_graph);
    UndirAdjList::vertex_descriptor v2 = boost::add_vertex(undir_graph);
    UndirAdjList::vertex_descriptor v3 = boost::add_vertex(undir_graph);

    // Add edges
    boost::add_edge(v0, v1, undir_graph);
    boost::add_edge(v0, v2, undir_graph);
    boost::add_edge(v1, v3, undir_graph);
    boost::add_edge(v2, v3, undir_graph);

    boost::remove_edge(v0, v1, undir_graph);

    // Check created graph
    ASSERT_TRUE((isValidStructure<UndirAdjList, kN>(undir_graph,
                                                    undir_graph_neighbours)));
}

// Verifying the creation of the described directed adjacency list
TEST(GraphCreation, DirAdjList) {
    constexpr uint64_t kN = 4;
    DirAdjList dir_graph(kN);

    std::array<std::vector<int>, kN> dir_graph_neighbours = {
        std::vector<int>{1, 2},
        std::vector<int>{3},
        std::vector<int>{3},
        std::vector<int>{},
    };

    // Add edges
    boost::add_edge(0, 1, dir_graph);
    boost::add_edge(0, 2, dir_graph);
    boost::add_edge(1, 3, dir_graph);
    boost::add_edge(2, 3, dir_graph);

    // Check created graph
    ASSERT_TRUE(
        (isValidStructure<DirAdjList, kN>(dir_graph, dir_graph_neighbours)));
}

// Verifying the creation of the described undirected adjacency matrix
TEST(GraphCreation, UndirAdjMatrix) {
    constexpr uint64_t kN = 4;

    UndirAdjMatrix undir_adj_matrix(kN);

    std::array<std::vector<int>, kN> undir_graph_neighbours = {
        std::vector<int>{1, 2},
        std::vector<int>{0, 3},
        std::vector<int>{0, 3},
        std::vector<int>{1, 2},
    };

    // Add edges
    boost::add_edge(0, 1, undir_adj_matrix);
    boost::add_edge(0, 2, undir_adj_matrix);
    boost::add_edge(1, 3, undir_adj_matrix);
    boost::add_edge(2, 3, undir_adj_matrix);

    // Check created graph
    ASSERT_TRUE((isValidStructure<UndirAdjMatrix, kN>(undir_adj_matrix,
                                                      undir_graph_neighbours)));
}

// Verifying the creation of the described directed adjacency matrix
TEST(GraphCreation, DirAdjMatrix) {
    constexpr uint64_t kN = 4;
    DirAdjMatrix dir_graph(kN);

    std::array<std::vector<int>, kN> dir_graph_neighbours = {
        std::vector<int>{1, 2},
        std::vector<int>{3},
        std::vector<int>{3},
        std::vector<int>{},
    };

    // Add edges
    boost::add_edge(0, 1, dir_graph);
    boost::add_edge(0, 2, dir_graph);
    boost::add_edge(1, 3, dir_graph);
    boost::add_edge(2, 3, dir_graph);

    // Check created graph
    ASSERT_TRUE(
        (isValidStructure<DirAdjMatrix, kN>(dir_graph, dir_graph_neighbours)));
}

// Union of two graphs
TEST(GraphCreation, CorrectGraphUnion) {
    UndirAdjList graph1(3);
    UndirAdjList graph2(6);

    UndirAdjList::vertex_descriptor g1_v3 = boost::add_vertex(graph1);
    UndirAdjList::vertex_descriptor g2_v6 = boost::add_vertex(graph2);

    // Add edges to graph1
    boost::add_edge(0, 1, graph1);
    boost::add_edge(0, 2, graph1);
    boost::add_edge(1, 3, graph1);
    boost::add_edge(2, 3, graph1);

    // Add edges to graph2
    boost::add_edge(0, 1, graph2);
    boost::add_edge(1, 6, graph2);
    boost::add_edge(1, 2, graph2);
    boost::add_edge(2, 3, graph2);
    boost::add_edge(3, 6, graph2);
    boost::add_edge(2, 4, graph2);
    boost::add_edge(2, 5, graph2);

    mergeGraphs<UndirAdjList, UndirAdjList::vertex_descriptor>(graph1, g1_v3,
                                                               graph2, g2_v6);

    std::array<std::vector<int>, 11> undir_graph_neighbours = {
        std::vector<int>{1, 2},       std::vector<int>{0, 3},
        std::vector<int>{0, 3},       std::vector<int>{1, 2, 10},
        std::vector<int>{5},          std::vector<int>{4, 6, 10},
        std::vector<int>{5, 7, 8, 9}, std::vector<int>{6, 10},
        std::vector<int>{6},          std::vector<int>{6},
        std::vector<int>{3, 5, 7}};

    // Check created graph
    ASSERT_TRUE(
        (isValidStructure<UndirAdjList, 11>(graph1, undir_graph_neighbours)));
}

// Trying to merge two graphs by adding an edge between them
TEST(GraphCreation, WrongGraphUnion) {
    UndirAdjList graph1(3);
    UndirAdjList graph2(6);

    UndirAdjList::vertex_descriptor g1_v3 = boost::add_vertex(graph1);
    UndirAdjList::vertex_descriptor g2_v6 = boost::add_vertex(graph2);

    // Add edges to graph1
    boost::add_edge(0, 1, graph1);
    boost::add_edge(0, 2, graph1);
    boost::add_edge(1, 3, graph1);
    boost::add_edge(2, 3, graph1);

    // Add edges to graph2
    boost::add_edge(0, 1, graph2);
    boost::add_edge(1, 6, graph2);
    boost::add_edge(1, 2, graph2);
    boost::add_edge(2, 3, graph2);
    boost::add_edge(3, 6, graph2);
    boost::add_edge(2, 4, graph2);
    boost::add_edge(2, 5, graph2);

    // Trying to add edge between vertices of different graphs should throw
    // exception, BUT IT'S NOT!!!
    EXPECT_THROW(boost::add_edge(g1_v3, g2_v6, graph1), std::exception);
}

// Checking the creation of graph properties
TEST(GraphProperty, Creation) {
    GraphWithProperties graph;

    // Add vertices with properties
    GraphWithProperties::vertex_descriptor v1 = boost::add_vertex(graph);
    GraphWithProperties::vertex_descriptor v2 = boost::add_vertex(graph);
    GraphWithProperties::vertex_descriptor v3 = boost::add_vertex(graph);

    graph[v1].name_ = "Vertex 0";
    graph[v2].name_ = "Vertex 1";
    graph[v3].name_ = "Vertex 2";

    boost::add_edge(v1, v2, EdgeProperties(10), graph);
    boost::add_edge(v2, v3, EdgeProperties(20), graph);
    boost::add_edge(v1, v3, EdgeProperties(30), graph);

    std::vector<std::string> vertex_name = {"Vertex 0", "Vertex 1", "Vertex 2"};
    std::vector<int> edge_weight = {10, 20, 30};

    ASSERT_TRUE((isValidVertexProperty<GraphWithProperties, VertexProperties,
                                       std::string>(graph, vertex_name)));

    ASSERT_TRUE((isValidEdgeProperty<GraphWithProperties, EdgeProperties, int>(
        graph, edge_weight)));
}

// Testing the topological sorting algorithm on directed graph
TEST(GraphAlgorighms, DirTopologicalSort) {
    typedef boost::graph_traits<DirAdjList>::vertex_descriptor VertexDescriptor;

    const std::size_t vertex_count = 7;
    const std::size_t edge_count = 6;

    const std::array<std::pair<int, int>, edge_count> edges = {
        {{0, 1}, {0, 2}, {1, 3}, {1, 4}, {2, 5}, {2, 6}}};

    DirAdjList g(edges.cbegin(), edges.cend(), vertex_count);

    std::array<VertexDescriptor, vertex_count> result_order;
    std::array<uint64_t, vertex_count> correct_order = {3, 4, 1, 5, 6, 2, 0};

    boost::topological_sort(g, result_order.begin());

    std::size_t ind = 0;
    for (const VertexDescriptor& v : result_order) {
        EXPECT_EQ(v, correct_order[ind++]);
    }
    EXPECT_EQ(ind, correct_order.size());
}

// Testing the topological sorting algorithm on undirected graph
TEST(GraphAlgorighms, UndirTopologicalSort) {
    typedef boost::graph_traits<UndirAdjList>::vertex_descriptor
        VertexDescriptor;

    const std::size_t vertex_count = 7;
    const std::size_t edge_count = 6;

    const std::array<std::pair<int, int>, edge_count> edges = {
        {{0, 1}, {0, 2}, {1, 3}, {1, 4}, {2, 5}, {2, 6}}};

    UndirAdjList g(edges.cbegin(), edges.cend(), vertex_count);

    std::array<VertexDescriptor, vertex_count> result_order;

    EXPECT_THROW(boost::topological_sort(g, result_order.begin()),
                 boost::not_a_dag);
}

// Testing the depth-first search algorithm
TEST(GraphAlgorighms, DepthFirstSearch) {
    const std::size_t vertex_count = 7;
    const std::size_t edge_count = 6;

    const std::array<std::pair<int, int>, edge_count> edges = {
        {{0, 1}, {0, 2}, {1, 3}, {1, 4}, {2, 5}, {2, 6}}};

    UndirAdjList g(edges.cbegin(), edges.cend(), vertex_count);
    DfsVisitorImpl visitor(vertex_count);

    std::array<uint64_t, vertex_count> correct_order = {0, 1, 3, 4, 2, 5, 6};
    boost::depth_first_search(g, boost::visitor(visitor).root_vertex(0u));

    std::size_t ind = 0;
    for (const auto& v : DfsVisitorImpl::visit_order) {
        EXPECT_EQ(v, correct_order[ind++]);
    }
    EXPECT_EQ(ind, correct_order.size());
}

// Testing the Dijkstra algorithm
TEST(GraphAlgorighms, Dijkstra) {
    WeightedGraph graph(6);

    add_edge(0, 1, graph);
    add_edge(1, 2, graph);
    add_edge(1, 3, graph);
    add_edge(3, 4, graph);
    add_edge(4, 5, graph);
    add_edge(2, 3, graph);

    boost::property_map<WeightedGraph, boost::edge_weight_t>::type weight_map =
        get(boost::edge_weight, graph);

    put(weight_map, edge(0, 1, graph).first, 2);
    put(weight_map, edge(1, 2, graph).first, 4);
    put(weight_map, edge(1, 3, graph).first, 6);
    put(weight_map, edge(2, 3, graph).first, 1);
    put(weight_map, edge(3, 4, graph).first, 3);
    put(weight_map, edge(4, 5, graph).first, 2);

    std::vector<int> dist(num_vertices(graph), std::numeric_limits<int>::max());
    std::vector<WeightedGraph::vertex_descriptor> parent(num_vertices(graph),
                                                         -1);
    dijkstra_shortest_paths(
        graph, 0, boost::predecessor_map(&parent[0]).distance_map(&dist[0]));

    std::vector<int> correct_dist = {0, 2, 6, 7, 10, 12};

    std::size_t ind = 0;
    for (const auto& curr_dist : dist) {
        EXPECT_EQ(curr_dist, correct_dist[ind++]);
    }
    EXPECT_EQ(ind, correct_dist.size());
}

// Checking for deletion of isolated vertex
TEST(GraphModification, DeleteIsolatedNode) {
    UndirAdjList undir_graph(11);

    boost::add_edge(0, 1, undir_graph);
    boost::add_edge(0, 2, undir_graph);
    boost::add_edge(1, 3, undir_graph);
    boost::add_edge(2, 3, undir_graph);
    boost::add_edge(4, 5, undir_graph);
    boost::add_edge(3, 10, undir_graph);
    boost::add_edge(5, 10, undir_graph);
    boost::add_edge(5, 6, undir_graph);
    boost::add_edge(7, 10, undir_graph);
    boost::add_edge(6, 7, undir_graph);
    boost::add_edge(6, 8, undir_graph);
    boost::add_edge(6, 9, undir_graph);

    boost::remove_edge(6, 7, undir_graph);
    boost::remove_edge(7, 10, undir_graph);

    boost::remove_vertex(7, undir_graph);

    std::array<std::vector<int>, 10> undir_graph_neighbours = {
        std::vector<int>{1, 2},    std::vector<int>{0, 3},
        std::vector<int>{0, 3},    std::vector<int>{1, 2, 9},
        std::vector<int>{5},       std::vector<int>{4, 6, 9},
        std::vector<int>{5, 7, 8}, std::vector<int>{6},
        std::vector<int>{6},       std::vector<int>{3, 5}};

    ASSERT_TRUE((isValidStructure<UndirAdjList, 10>(undir_graph,
                                                    undir_graph_neighbours)));
}

// Checking for deletion of non-isolated vertex
// NOTE: incident ribs are not removed
TEST(GraphModification, DeleteNonIsolatedNode) {
    UndirAdjList undir_graph(11);

    boost::add_edge(0, 1, undir_graph);
    boost::add_edge(0, 2, undir_graph);
    boost::add_edge(1, 3, undir_graph);
    boost::add_edge(2, 3, undir_graph);
    boost::add_edge(4, 5, undir_graph);
    boost::add_edge(3, 10, undir_graph);
    boost::add_edge(5, 10, undir_graph);
    boost::add_edge(5, 6, undir_graph);
    boost::add_edge(7, 10, undir_graph);
    boost::add_edge(6, 7, undir_graph);
    boost::add_edge(6, 8, undir_graph);
    boost::add_edge(6, 9, undir_graph);

    boost::remove_vertex(10, undir_graph);

    std::array<std::vector<int>, 10> undir_graph_neighbours = {
        std::vector<int>{1, 2},       std::vector<int>{0, 3},
        std::vector<int>{0, 3},       std::vector<int>{1, 2},
        std::vector<int>{5},          std::vector<int>{4, 6},
        std::vector<int>{5, 7, 8, 9}, std::vector<int>{6},
        std::vector<int>{6},          std::vector<int>{6}};

    ASSERT_TRUE((isValidStructure<UndirAdjList, 10>(undir_graph,
                                                    undir_graph_neighbours)));
}

// Creation of induced subgraphs
TEST(InducedSubgraph, CreateInducedSubgraph) {
    Subgraph graph(6);

    boost::add_edge(0, 1, graph);
    boost::add_edge(1, 2, graph);
    boost::add_edge(1, 3, graph);
    boost::add_edge(1, 4, graph);
    boost::add_edge(3, 5, graph);
    boost::add_edge(4, 5, graph);

    Subgraph& subgraph1 = graph.create_subgraph();
    Subgraph& subgraph2 = graph.create_subgraph();

    boost::add_vertex(0, subgraph1);
    boost::add_vertex(1, subgraph1);

    boost::add_vertex(2, subgraph2);
    boost::add_vertex(4, subgraph2);
    boost::add_vertex(5, subgraph2);

    std::array<std::vector<int>, 2> undir_subgraph1_neighbours = {
        std::vector<int>{1}, std::vector<int>{0}};

    std::array<std::vector<int>, 3> undir_subgraph2_neighbours = {
        std::vector<int>{}, std::vector<int>{2}, std::vector<int>{1}};

    ASSERT_TRUE(
        (isValidStructure<Subgraph, 2>(subgraph1, undir_subgraph1_neighbours)));

    ASSERT_TRUE(
        (isValidStructure<Subgraph, 3>(subgraph2, undir_subgraph2_neighbours)));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}