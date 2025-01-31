#include <gtest/gtest.h>
#include <chrono>
#include <memory>

#include <boost/graph/exception.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "../src/graph_validation/include/graph_validation.hpp"

class BfsVisitorImpl : public boost::default_bfs_visitor {
public:
    explicit BfsVisitorImpl()
        : boost::default_bfs_visitor(),
          sh_ptr_vec_(std::make_shared<std::vector<int>>()) {}

    template <class Vertex, class Graph>
    void discover_vertex(Vertex u, Graph&) {
        (*sh_ptr_vec_).push_back(static_cast<int>(u));
    }

    const std::shared_ptr<std::vector<int>> sh_ptr_vec_;
};

TEST(GraphCreation, UndirAdjList_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/undirected_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nCREATION UNDIRECTED ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        undir_graph, "../tests/output/undirected_10_struct.txt")));
    input_file.close();
}

TEST(GraphCreation, UndirAdjList_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/undirected_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION UNDIRECTED ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        undir_graph, "../tests/output/undirected_100_struct.txt")));
    input_file.close();
}

TEST(GraphCreation, UndirAdjList_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/undirected_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION UNDIRECTED ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        undir_graph, "../tests/output/undirected_1000_struct.txt")));
    input_file.close();
}

TEST(GraphCreation, SparseUndirAsjList_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/sparse_graph_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nCREATION SPARSE GRAPH ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphCreation, SparseUndirAsjList_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/sparse_graph_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION SPARSE GRAPH ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphCreation, SparseUndirAsjList_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/sparse_graph_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION SPARSE GRAPH ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphCreation, DirAdjList_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/directed_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION DIRECTED ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        dir_graph, "../tests/output/directed_10_struct.txt")));
    input_file.close();
}

TEST(GraphCreation, DirAdjList_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/directed_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION DIRECTED ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        dir_graph, "../tests/output/directed_100_struct.txt")));
    input_file.close();
}

TEST(GraphCreation, DirAdjList_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/directed_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    auto start = std::chrono::steady_clock::now();
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    auto end = std::chrono::steady_clock::now();
    std::cout << "\n CREATION DIRECTED ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidGraphStructure(
        dir_graph, "../tests/output/directed_1000_struct.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, TopSort_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/topsort_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nTOPSORT ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE(
        (isValidTopSort(result_order, "../tests/output/topsort_10.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, TopSort_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/topsort_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nTOPSORT ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE(
        (isValidTopSort(result_order, "../tests/output/topsort_100.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, TopSort_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/topsort_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nTOPSORT ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE(
        (isValidTopSort(result_order, "../tests/output/topsort_1000.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, SparseTopSort10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/sparse_topsort_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE TOPSORT ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseTopSort100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/sparse_topsort_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE TOPSORT ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseTopSort1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/sparse_topsort_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    DirAdjList dir_graph(edges.begin(), edges.end(), vertices);
    std::array<DirAdjListVertexDesc, vertices> result_order;
    auto start = std::chrono::steady_clock::now();
    boost::topological_sort(dir_graph, result_order.begin());
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE TOPSORT ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, Dijkstra_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/undirected_10.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nDIJKSTRA ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidAlgAns(dist, "../tests/output/dijkstra_10.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, Dijkstra_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/undirected_100.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nDIJKSTRA ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidAlgAns(dist, "../tests/output/dijkstra_100.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, Dijkstra_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/undirected_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nDIJKSTRA ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";

    ASSERT_TRUE((isValidAlgAns(dist, "../tests/output/dijkstra_1000.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, SparseDijkstra_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/sparse_graph_10.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nSPARSE DIJKSTRA ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseDijkstra_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/sparse_graph_100.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nSPARSE DIJKSTRA ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseDijkstra_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/sparse_graph_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    WeightedAdjList weighted_graph(vertices);
    uint64_t vertex1 = 0;
    uint64_t vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        boost::add_edge(vertex1, vertex2, 1, weighted_graph);
    }
    std::vector<int> dist(num_vertices(weighted_graph),
                          std::numeric_limits<int>::max());
    std::vector<WeightedAdjList::vertex_descriptor> parent(
        num_vertices(weighted_graph), std::numeric_limits<uint64_t>::max());
    auto start = std::chrono::steady_clock::now();
    dijkstra_shortest_paths(
        weighted_graph, 0,
        boost::predecessor_map(&parent[0]).distance_map(&dist[0]));
    auto end = std::chrono::steady_clock::now();

    std::cout << "\nSPARSE DIJKSTRA ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, BFS_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/undirected_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nBFS ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    ASSERT_TRUE(
        (isValidAlgAns(*(visitor.sh_ptr_vec_), "../tests/output/bfs_10.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, BFS_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/undirected_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nBFS ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    ASSERT_TRUE(
        (isValidAlgAns(*(visitor.sh_ptr_vec_), "../tests/output/bfs_100.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, BFS_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/undirected_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nBFS ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    ASSERT_TRUE((
        isValidAlgAns(*(visitor.sh_ptr_vec_), "../tests/output/bfs_1000.txt")));
    input_file.close();
}

TEST(GraphAlgorithms, SparseBFS_10) {
    const int vertices = 10;
    std::ifstream input_file("../tests/input/sparse_graph_10.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE BFS ADJACENTY LIST 10 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseBFS_100) {
    const int vertices = 100;
    std::ifstream input_file("../tests/input/sparse_graph_100.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE BFS ADJACENTY LIST 100 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

TEST(GraphAlgorithms, SparseBFS_1000) {
    const int vertices = 1000;
    std::ifstream input_file("../tests/input/sparse_graph_1000.txt");
    ASSERT_TRUE(input_file.is_open());
    std::vector<std::pair<int, int>> edges;
    edges.reserve(vertices);
    int vertex1 = 0;
    int vertex2 = 0;
    while (input_file.peek() != EOF) {
        input_file >> vertex1;
        input_file >> vertex2;
        edges.push_back(std::pair<int, int>{vertex1, vertex2});
    }
    UndirAdjList undir_graph(edges.begin(), edges.end(), vertices);
    BfsVisitorImpl visitor;
    auto start = std::chrono::steady_clock::now();
    boost::breadth_first_search(undir_graph, 0, boost::visitor(visitor));
    auto end = std::chrono::steady_clock::now();
    std::cout << "\nSPARSE BFS ADJACENTY LIST 1000 NODES\n";
    std::cout << "Time: "
              << std::chrono::duration<double, std::milli>(end - start).count()
              << " milliseconds" << "\n\n";
    input_file.close();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
