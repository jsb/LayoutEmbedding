#pragma once

#include <polymesh/pm.hh>

#include <variant>
#include <vector>

namespace LayoutEmbedding {

using VirtualVertex = std::variant<
    pm::vertex_handle,
    pm::edge_handle
>;

bool is_valid(const VirtualVertex& _vv);

bool is_real_vertex(const VirtualVertex& _el);
bool is_real_edge(const VirtualVertex& _el);

// Warning: These will throw when the contained element does not match.
pm::vertex_handle real_vertex(const VirtualVertex& _el);
pm::edge_handle real_edge(const VirtualVertex& _el);

}
