#include "include/graph_validation.hpp"

template <typename Graph>
[[nodiscard]] bool isValidGraphStructure(
    const Graph& graph, const std::string& correct_output_file_path) {
    std::ifstream input_file(correct_output_file_path);
    if (!input_file.is_open()) {
        return false;
    }
    typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    int curr_vertex = 0;
    for (boost::tie(vi, vi_end) = boost::vertices(graph); vi != vi_end; ++vi) {
        for (auto vd :
             boost::make_iterator_range(boost::adjacent_vertices(*vi, graph))) {
            input_file >> curr_vertex;
            if (vd != static_cast<unsigned int>(curr_vertex)) {
                input_file.close();
                return false;
            }
        }
        if (int next_char_id = input_file.peek();
            next_char_id != 10 && next_char_id != EOF) {
            input_file.close();
            return false;
        }
    }
    input_file.close();
    return true;
}

[[nodiscard]] bool isValidAlgAns(const std::vector<int>& answer_array,
                                 const std::string& correct_output_file_path) {
    std::ifstream input_file(correct_output_file_path);
    if (!input_file.is_open()) {
        return false;
    }
    int correct_curr_vertex = 0;
    for (const int& ans_vertex : answer_array) {
        input_file >> correct_curr_vertex;
        if (correct_curr_vertex != ans_vertex) {
            input_file.close();
            return false;
        }
    }
    if (int next_char_id = input_file.peek();
        next_char_id != 10 && next_char_id != EOF) {
        input_file.close();
        return false;
    }
    input_file.close();
    return true;
}

template <uint64_t N>
[[nodiscard]] bool isValidTopSort(
    const std::array<DirAdjListVertexDesc, N>& answer_array,
    const std::string& correct_output_file_path) {
    std::ifstream input_file(correct_output_file_path);
    if (!input_file.is_open()) {
        return false;
    }
    int correct_curr_vertex = 0;
    for (const auto& ans_vertex : answer_array) {
        input_file >> correct_curr_vertex;
        if (static_cast<uint64_t>(correct_curr_vertex) != ans_vertex) {
            input_file.close();
            return false;
        }
    }
    if (int next_char_id = input_file.peek();
        next_char_id != 10 && next_char_id != EOF) {
        input_file.close();
        return false;
    }
    input_file.close();
    return true;
}

template bool isValidGraphStructure<UndirAdjList>(
    const UndirAdjList& graph, const std::string& correct_output_file_path);

template bool isValidGraphStructure<DirAdjList>(
    const DirAdjList& graph, const std::string& correct_output_file_path);

template bool isValidGraphStructure<UndirAdjMatrix>(
    const UndirAdjMatrix& graph, const std::string& correct_output_file_path);

template bool isValidGraphStructure<DirAdjMatrix>(
    const DirAdjMatrix& graph, const std::string& correct_output_file_path);

template bool isValidTopSort<10>(
    const std::array<DirAdjListVertexDesc, 10>& answer_array,
    const std::string& correct_output_file_path);

template bool isValidTopSort<100>(
    const std::array<DirAdjListVertexDesc, 100>& answer_array,
    const std::string& correct_output_file_path);

template bool isValidTopSort<1000>(
    const std::array<DirAdjListVertexDesc, 1000>& answer_array,
    const std::string& correct_output_file_path);
