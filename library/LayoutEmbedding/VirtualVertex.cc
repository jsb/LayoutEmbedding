#include "VirtualVertex.hh"

#include <LayoutEmbedding/Assert.hh>

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

void set_mesh(VirtualVertex& _vv, const pm::Mesh& _m)
{
    _vv = on_mesh(_vv, _m);
}

VirtualVertex on_mesh(const VirtualVertex& _vv, const pm::Mesh& _m)
{
    if (is_real_vertex(_vv)) {
        return VirtualVertex(_m[real_vertex(_vv).idx]);
    }
    else {
        return VirtualVertex(_m[real_edge(_vv).idx]);
    }
}

}
