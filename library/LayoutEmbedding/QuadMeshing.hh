#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

namespace LayoutEmbedding
{

struct Embedding;

/// Takes an embedded quad layout and returns the number of
/// subdivisions per edge that best achives the target edge length.
pm::edge_attribute<int> choose_loop_subdivisions(
        const Embedding& _em,
        const double _target_edge_length);

/// Takes an embedded quad layout and a valid number of subdivisions
/// per edge. Returns an integer-grid map.
pm::halfedge_attribute<tg::dpos2> parametrize_patches(
        const Embedding& _em,
        const pm::edge_attribute<int>& _l_subdivisions);

}
