#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

void make_layout_by_decimation(
    const pm::vertex_attribute<tg::pos3>& _t_pos,
    int _n_vertices,
    pm::Mesh& _l_m, // output
    pm::vertex_attribute<tg::pos3>& _l_pos
);
