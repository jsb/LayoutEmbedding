#include "VirtualVertex.hh"

#include <LayoutEmbedding/Util/Assert.hh>

namespace LayoutEmbedding {

bool is_valid(const VirtualVertex& _vv)
{
    if (is_real_vertex(_vv)) {
        return real_vertex(_vv).is_valid();
    }
    else {
        return real_edge(_vv).is_valid();
    }
}

bool is_real_vertex(const VirtualVertex& _el)
{
    return std::holds_alternative<pm::vertex_index>(_el);
}

bool is_real_edge(const VirtualVertex& _el)
{
    return std::holds_alternative<pm::edge_index>(_el);
}

pm::vertex_index real_vertex(const VirtualVertex& _el)
{
    LE_ASSERT(is_real_vertex(_el));
    return std::get<pm::vertex_index>(_el);
}

pm::edge_index real_edge(const VirtualVertex& _el)
{
    LE_ASSERT(is_real_edge(_el));
    return std::get<pm::edge_index>(_el);
}

pm::vertex_handle real_vertex(const VirtualVertex& _el, const pm::Mesh& _on_mesh)
{
    return _on_mesh[real_vertex(_el)];
}

pm::edge_handle real_edge(const VirtualVertex& _el, const pm::Mesh& _on_mesh)
{
    return _on_mesh[real_edge(_el)];
}

}
