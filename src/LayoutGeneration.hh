#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

void make_layout_by_decimation(
    const pm::vertex_attribute<tg::pos3>& _t_pos,
    int _n_vertices,
    pm::Mesh& _l_m, // output
    pm::vertex_attribute<tg::pos3>& _l_pos
);

using MatchingVertices = std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>>;

MatchingVertices find_matching_vertices(
    const pm::vertex_attribute<tg::pos3>& _l_pos,
    const pm::vertex_attribute<tg::pos3>& _t_pos
);

void jitter_matching_vertices(
    const pm::Mesh& _l_m,
    const pm::Mesh& _t_m,
    MatchingVertices& _mv,
    int _steps = 1
);
