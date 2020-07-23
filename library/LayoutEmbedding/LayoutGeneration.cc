#include "LayoutGeneration.hh"

#include <polymesh/algorithms/decimate.hh>

#include <set>

namespace LayoutEmbedding {

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

MatchingVertices find_matching_vertices(const pm::vertex_attribute<tg::pos3>& _l_pos, const pm::vertex_attribute<tg::pos3>& _t_pos)
{
    const pm::Mesh& l_m = _l_pos.mesh();
    const pm::Mesh& t_m = _t_pos.mesh();

    std::set<pm::vertex_index> t_matched_v_ids;

    MatchingVertices result;
    for (const auto& l_v : l_m.vertices()) {
        auto best_distance_sqr = tg::inf<double>;
        auto best_t_v = pm::vertex_handle::invalid;
        for (const auto& t_v : t_m.vertices()) {
            // Don't match the same target vertex twice
            if (t_matched_v_ids.count(t_v.idx)) {
                continue;
            }
            const auto distance_sqr = tg::distance_sqr(_l_pos[l_v], _t_pos[t_v]);
            if (distance_sqr < best_distance_sqr) {
                best_t_v = t_v;
                best_distance_sqr = distance_sqr;
            }
        }
        result.emplace_back(l_v, best_t_v);
        t_matched_v_ids.insert(best_t_v.idx);
    }
    return result;
}

void jitter_matching_vertices(const pm::Mesh& _l_m, const pm::Mesh& _t_m, MatchingVertices& _mv, int _steps)
{
    tg::rng rng;

    pm::vertex_attribute<bool> t_v_occupied(_t_m);
    for (const auto& [l_v, t_v] : _mv) {
        t_v_occupied[t_v] = true;
    }

    for (int i = 0; i < _steps; ++i) {
        for (auto& [l_v, t_v] : _mv) {
            auto t_v_new = t_v.adjacent_vertices().random(rng);
            if (!t_v_occupied[t_v_new]) {
                t_v_occupied[t_v] = false;
                t_v_occupied[t_v_new] = true;
                t_v = t_v_new;
            }
        }
    }
}

}
