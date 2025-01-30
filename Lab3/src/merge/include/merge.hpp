#ifndef LAB3_SRC_MERGE
#define LAB3_SRC_MERGE

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/copy.hpp>

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>
    UndirAdjList;

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS>
    DirAdjList;

typedef boost::adjacency_matrix<boost::undirectedS> UndirAdjMatrix;

typedef boost::adjacency_matrix<boost::directedS> DirAdjMatrix;

template <typename Graph, typename VertexDesc>
void mergeGraphs(Graph& g1, VertexDesc v_in_g1, const Graph& g2,
                 VertexDesc u_in_g2);

#endif  // LAB3_SRC_MERGE
