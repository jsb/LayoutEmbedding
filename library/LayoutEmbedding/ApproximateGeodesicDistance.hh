#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg-lean.hh>

namespace LayoutEmbedding {

pm::vertex_attribute<double>
approximate_geodesic_distance(
    const pm::vertex_attribute<tg::pos3>& _pos,
    const std::vector<pm::vertex_handle>& _source_vertices
);

}
