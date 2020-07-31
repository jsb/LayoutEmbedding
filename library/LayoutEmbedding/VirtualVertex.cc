#include "VirtualVertex.hh"

#include <LayoutEmbedding/Assert.hh>

namespace LayoutEmbedding {

bool is_real_vertex(const VirtualVertex& _el)
{
    return std::holds_alternative<pm::vertex_handle>(_el);
}

bool is_real_edge(const VirtualVertex& _el)
{
    return std::holds_alternative<pm::edge_handle>(_el);
}

polymesh::vertex_handle real_vertex(const VirtualVertex& _el)
{
    LE_ASSERT(is_real_vertex(_el));
    return std::get<pm::vertex_handle>(_el);
}

polymesh::edge_handle real_edge(const VirtualVertex& _el)
{
    LE_ASSERT(is_real_edge(_el));
    return std::get<pm::edge_handle>(_el);
}

}
