#pragma once

#include <LayoutEmbedding/Parametrization.hh>

namespace LayoutEmbedding
{

struct Embedding;

/// Takes an embedded quad layout and returns the number of
/// subdivisions per edge that best achives the target edge length.
pm::edge_attribute<int> choose_loop_subdivisions(
        const Embedding& _em,
        const double _target_edge_length,
        const int _max = INT_MAX);

/// Takes an embedded quad layout and a valid number of subdivisions
/// per edge. Returns an integer-grid map.
HalfedgeParam parametrize_patches(
        const Embedding& _em,
        const pm::edge_attribute<int>& _l_subdivisions);

/// Takes an integer-grid map and extracts a quad mesh.
pm::vertex_attribute<tg::pos3> extract_quad_mesh(
        const Embedding& _em,
        HalfedgeParam _param, // copy
        const double _param_scale,
        pm::Mesh& _q,
        pm::face_attribute<pm::face_handle>& _q_matching_layout_face);

}
