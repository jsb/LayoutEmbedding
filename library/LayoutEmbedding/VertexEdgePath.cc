#include "VertexEdgePath.hh"

#include <LayoutEmbedding/Assert.hh>

namespace LayoutEmbedding {

bool is_vertex(const VertexEdgeElement& _el)
{
    return std::holds_alternative<pm::vertex_handle>(_el);
}

bool is_edge(const VertexEdgeElement& _el)
{
    return std::holds_alternative<pm::edge_handle>(_el);
}

polymesh::vertex_handle vertex(const VertexEdgeElement& _el)
{
    LE_ASSERT(is_vertex(_el));
    return std::get<pm::vertex_handle>(_el);
}

polymesh::edge_handle edge(const VertexEdgeElement& _el)
{
    LE_ASSERT(is_edge(_el));
    return std::get<pm::edge_handle>(_el);
}

}
