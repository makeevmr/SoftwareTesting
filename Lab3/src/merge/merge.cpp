#include "include/merge.hpp"

template <typename Graph, typename VertexDesc>
void mergeGraphs(Graph& g1, VertexDesc v_in_g1, const Graph& g2,
                 VertexDesc u_in_g2) {
    using index_map_t = boost::property_map<Graph, boost::vertex_index_t>::type;
    using vertex_t = Graph::vertex_descriptor;

    typedef boost::iterator_property_map<
        typename std::vector<vertex_t>::iterator, index_map_t, vertex_t,
        vertex_t&>
        IsoMap;

    std::vector<vertex_t> orig2copy_data(num_vertices(g2));
    IsoMap map_v = make_iterator_property_map(orig2copy_data.begin(),
                                              get(boost::vertex_index, g2));

    boost::copy_graph(g2, g1, boost::orig_to_copy(map_v));

    vertex_t u_in_g1 = map_v[u_in_g2];
    boost::add_edge(v_in_g1, u_in_g1, g1);
}

template void mergeGraphs<UndirAdjList, UndirAdjList::vertex_descriptor>(
    UndirAdjList& g1, UndirAdjList::vertex_descriptor v_in_g1,
    const UndirAdjList& g2, UndirAdjList::vertex_descriptor u_in_g2);
