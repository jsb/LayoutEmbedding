#pragma once

#include <polymesh/pm.hh>

#include <variant>
#include <vector>

namespace LayoutEmbedding {

using VirtualVertex = std::variant<
    pm::vertex_handle,
    pm::edge_handle
>;

bool is_real_vertex(const VirtualVertex& _el);
bool is_real_edge(const VirtualVertex& _el);

// Warning: These will throw when the contained element does not match.
pm::vertex_handle real_vertex(const VirtualVertex& _el);
pm::edge_handle real_edge(const VirtualVertex& _el);

using VirtualPath = std::vector<VirtualVertex>;

template <typename T>
struct VirtualVertexAttribute
{
    pm::vertex_attribute<T> v_a;
    pm::edge_attribute<T> e_a;

    VirtualVertexAttribute(const pm::Mesh& _m) :
        v_a(_m),
        e_a(_m)
    {
    }

    T& operator[](const VirtualVertex& _el)
    {
        if (is_real_vertex(_el)) {
            return v_a[real_vertex(_el)];
        }
        else {
            return e_a[real_edge(_el)];
        }
    }
};

}
