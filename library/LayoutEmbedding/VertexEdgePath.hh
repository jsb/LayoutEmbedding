#pragma once

#include <polymesh/pm.hh>

#include <variant>
#include <vector>

using VertexEdgeElement = std::variant<
    pm::vertex_handle,
    pm::edge_handle
>;

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

    T& operator[](const VertexEdgeElement& _ve)
    {
        if (const pm::vertex_handle* v = std::get_if<pm::vertex_handle>(&_ve)) {
            return v_a[*v];
        }
        else if (const pm::edge_handle* e = std::get_if<pm::edge_handle>(&_ve)) {
            return e_a[*e];
        }
    }
};
