#include "LayoutGeneration.hh"

#include <LayoutEmbedding/Assert.hh>

#include <polymesh/algorithms/decimate.hh>

#include <set>

namespace LayoutEmbedding {

void make_layout_by_decimation(EmbeddingInput& _input, int _n_vertices)
{
    LE_ASSERT(_input.t_m.vertices().size() > 0);

    _input.l_m.copy_from(_input.t_m);
    _input.l_pos.copy_from(_input.t_pos);

    pm::vertex_attribute<tg::quadric3> l_error(_input.l_m);
    for (const auto& l_v : _input.l_m.vertices()) {
        for (const auto& l_f : l_v.faces()) {
            const auto& p = _input.l_pos[l_v];
            const auto& n = pm::face_normal(l_f, _input.l_pos);
            l_error[l_v].add_plane(p, n, 0.0);
        }
    }

    pm::decimate_down_to(_input.l_m, _input.l_pos, l_error, _n_vertices);
    _input.l_m.compactify();
}

void find_matching_vertices_by_proximity(EmbeddingInput& _input)
{
    std::set<pm::vertex_index> t_matched_v_ids;

    for (const auto& l_v : _input.l_m.vertices()) {
        auto best_distance_sqr = tg::inf<double>;
        auto best_t_v = pm::vertex_handle::invalid;
        for (const auto& t_v : _input.t_m.vertices()) {
            // Don't match the same target vertex twice
            if (t_matched_v_ids.count(t_v.idx)) {
                continue;
            }
            const auto distance_sqr = tg::distance_sqr(_input.l_pos[l_v], _input.t_pos[t_v]);
            if (distance_sqr < best_distance_sqr) {
                best_t_v = t_v;
                best_distance_sqr = distance_sqr;
            }
        }
        LE_ASSERT(best_t_v.is_valid());
        _input.l_matching_vertex[l_v] = best_t_v;
        t_matched_v_ids.insert(best_t_v.idx);
    }
}

void jitter_matching_vertices(EmbeddingInput& _input, int _steps, int _seed)
{
    tg::rng rng;
    rng.seed(_seed);

    pm::vertex_attribute<bool> t_v_occupied(_input.t_m);
    for (const auto l_v : _input.l_m.vertices()) {
        const auto& t_v = _input.l_matching_vertex[l_v];
        LE_ASSERT(t_v.is_valid());
        t_v_occupied[t_v] = true;
    }

    for (int i = 0; i < _steps; ++i) {
        for (const auto l_v : _input.l_m.vertices()) {
            const auto& t_v = _input.l_matching_vertex[l_v];
            const auto t_v_new = t_v.adjacent_vertices().random(rng);
            if (!t_v_occupied[t_v_new]) {
                t_v_occupied[t_v] = false;
                t_v_occupied[t_v_new] = true;
                _input.l_matching_vertex[l_v] = t_v_new;
            }
        }
    }
}

void randomize_matching_vertices(EmbeddingInput& _input)
{
    std::vector<pm::vertex_handle> t_vertices = _input.t_m.vertices().to_vector();
    std::random_shuffle(t_vertices.begin(), t_vertices.end());

    for (int i = 0; i < _input.l_m.vertices().size(); ++i) {
        const auto& l_v = _input.l_m.vertices()[i];
        const auto& t_v = t_vertices[i];
        _input.l_matching_vertex[l_v] = t_v;
    }
}

}
