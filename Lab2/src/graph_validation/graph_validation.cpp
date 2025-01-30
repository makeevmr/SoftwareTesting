#include "include/graph_validation.hpp"

template <typename Graph, uint64_t N>
[[nodiscard]] bool isValidStructure(
    const Graph& graph, const std::array<std::vector<int>, N>& neighbours) {
    typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    std::size_t curr_vertex = 0;
    for (boost::tie(vi, vi_end) = boost::vertices(graph); vi != vi_end; ++vi) {
        std::size_t curr_neighbour = 0;
        for (auto vd :
             boost::make_iterator_range(boost::adjacent_vertices(*vi, graph))) {
            if (vd != static_cast<uint64_t>(
                          neighbours[curr_vertex][curr_neighbour])) {
                return false;
            }
            ++curr_neighbour;
        }
        if (curr_neighbour != neighbours[curr_vertex].size()) {
            return false;
        }
        ++curr_vertex;
    }
    return true;
}

template <typename Graph, typename VertexProperties, typename Property>
[[nodiscard]] bool isValidVertexProperty(
    Graph& graph, std::vector<Property>& correct_vertices_property) {
    typename boost::property_map<Graph, std::string VertexProperties::*>::type
        vertex_properties = get(&VertexProperties::name_, graph);
    typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
    std::size_t ind = 0;
    for (boost::tie(vi, vi_end) = boost::vertices(graph); vi != vi_end; ++vi) {
        if (vertex_properties[*vi] != correct_vertices_property[ind]) {
            return false;
        }
        ++ind;
    }
    return ind == correct_vertices_property.size();
}

template <typename Graph, typename EdgeProperties, typename Property>
[[nodiscard]] bool isValidEdgeProperty(
    Graph& graph, std::vector<Property>& correct_edges_property) {
    typename boost::property_map<Graph, int EdgeProperties::*>::type
        edge_properties = get(&EdgeProperties::weight_, graph);
    typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    std::size_t ind = 0;
    for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
        if (edge_properties[*ei] != correct_edges_property[ind]) {
            return false;
        }
        ++ind;
    }
    return ind == correct_edges_property.size();
}

template bool isValidStructure<UndirAdjList, 4>(
    const UndirAdjList& graph,
    const std::array<std::vector<int>, 4>& neighbours);

template bool isValidStructure<DirAdjList, 4>(
    const DirAdjList& graph, const std::array<std::vector<int>, 4>& neighbours);

template bool isValidStructure<UndirAdjMatrix, 4>(
    const UndirAdjMatrix& graph,
    const std::array<std::vector<int>, 4>& neighbours);

template bool isValidStructure<DirAdjMatrix, 4>(
    const DirAdjMatrix& graph,
    const std::array<std::vector<int>, 4>& neighbours);

template bool isValidStructure<UndirAdjList, 11>(
    const UndirAdjList& graph,
    const std::array<std::vector<int>, 11>& neighbours);

template bool isValidStructure<UndirAdjList, 10>(
    const UndirAdjList& graph,
    const std::array<std::vector<int>, 10>& neighbours);

template bool isValidStructure<Subgraph, 2>(
    const Subgraph& graph, const std::array<std::vector<int>, 2>& neighbours);

template bool isValidStructure<Subgraph, 3>(
    const Subgraph& graph, const std::array<std::vector<int>, 3>& neighbours);

template bool
isValidVertexProperty<GraphWithProperties, VertexProperties, std::string>(
    GraphWithProperties& graph,
    std::vector<std::string>& correct_edges_property);

template bool isValidEdgeProperty<GraphWithProperties, EdgeProperties, int>(
    GraphWithProperties& graph, std::vector<int>& correct_edges_property);
