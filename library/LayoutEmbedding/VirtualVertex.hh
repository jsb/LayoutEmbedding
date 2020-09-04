#pragma once

#include <polymesh/pm.hh>

#include <variant>
#include <vector>

namespace LayoutEmbedding {

using VirtualVertex = std::variant<
    pm::vertex_index,
    pm::edge_index
>;

bool is_valid(const VirtualVertex& _vv);

bool is_real_vertex(const VirtualVertex& _el);
bool is_real_edge(const VirtualVertex& _el);

// Warning: These will throw when the contained element does not match.
pm::vertex_index real_vertex(const VirtualVertex& _el);
pm::edge_index real_edge(const VirtualVertex& _el);

// Convenience overloads that allow providing a mesh to construct the handle on
pm::vertex_handle real_vertex(const VirtualVertex& _el, const pm::Mesh& _on_mesh);
pm::edge_handle real_edge(const VirtualVertex& _el, const pm::Mesh& _on_mesh);

}
