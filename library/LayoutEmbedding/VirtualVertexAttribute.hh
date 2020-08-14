#pragma once

#include <LayoutEmbedding/VirtualVertex.hh>

#include <polymesh/pm.hh>

namespace LayoutEmbedding {

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
