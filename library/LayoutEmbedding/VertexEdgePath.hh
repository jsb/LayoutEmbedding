#pragma once

#include <polymesh/pm.hh>

#include <variant>
#include <vector>

namespace LayoutEmbedding {

using VertexEdgeElement = std::variant<
    pm::vertex_handle,
    pm::edge_handle
>;

bool is_vertex(const VertexEdgeElement& _el);
bool is_edge(const VertexEdgeElement& _el);

// Warning: These will throw when the contained element does not match.
pm::vertex_handle vertex(const VertexEdgeElement& _el);
pm::edge_handle edge(const VertexEdgeElement& _el);

using VertexEdgePath = std::vector<VertexEdgeElement>;

template <typename T>
struct VertexEdgeAttribute
{
    pm::vertex_attribute<T> v_a;
    pm::edge_attribute<T> e_a;

    VertexEdgeAttribute(const pm::Mesh& _m) :
        v_a(_m),
        e_a(_m)
    {
    }

    T& operator[](const VertexEdgeElement& _el)
    {
        if (is_vertex(_el)) {
            return v_a[vertex(_el)];
        }
        else {
            return e_a[edge(_el)];
        }
    }
};

}
