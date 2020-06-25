#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

void make_layout_by_decimation(
    const pm::vertex_attribute<tg::pos3>& _t_pos,
    int _n_vertices,
    pm::Mesh& _l_m, // output
    pm::vertex_attribute<tg::pos3>& _l_pos
);

std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>> find_matching_vertices(
    const pm::vertex_attribute<tg::pos3>& _l_pos,
    const pm::vertex_attribute<tg::pos3>& _t_pos
);
