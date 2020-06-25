#include "layout_generation.hh"

#include <polymesh/algorithms/decimate.hh>

void make_layout_by_decimation(const pm::vertex_attribute<tg::pos3>& _t_pos, int _n_vertices, pm::Mesh& _l_m, pm::vertex_attribute<tg::pos3>& _l_pos)
{
    _l_m.copy_from(_t_pos.mesh());
    _l_pos.copy_from(_t_pos);

    pm::vertex_attribute<tg::quadric3> l_error(_l_m);
    for (const auto& l_v : _l_m.vertices()) {
        for (const auto& l_f : l_v.faces()) {
            const auto& p = _l_pos[l_v];
            const auto& n = pm::face_normal(l_f, _l_pos);
            l_error[l_v].add_plane(p, n, 0.0);
        }
    }

    pm::decimate_down_to(_l_m, _l_pos, l_error, _n_vertices);
    _l_m.compactify();
}
