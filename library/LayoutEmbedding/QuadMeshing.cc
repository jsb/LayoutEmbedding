#include "QuadMeshing.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Harmonic.hh>
#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding
{

namespace
{

void extract_patch(
        const Embedding& _em,
        const pm::face_handle& _l_f,
        pm::Mesh& _patch,
        pm::vertex_attribute<tg::pos3>& _patch_pos,
        pm::vertex_attribute<pm::vertex_handle>& _v_target_to_patch,
        pm::halfedge_attribute<pm::halfedge_handle>& _h_patch_to_target)
{
    // Init result
    _patch.clear();
    _patch_pos = _patch.vertices().make_attribute<tg::pos3>();

    // Index maps
    _v_target_to_patch = _em.target_mesh().vertices().make_attribute<pm::vertex_handle>();
    _h_patch_to_target = _patch.halfedges().make_attribute<pm::halfedge_handle>();

    // Create region mesh
    for (auto t_f : _em.get_patch(_l_f))
    {
        // Add vertices to result mesh
        for (auto t_v : t_f.vertices())
        {
            if (_v_target_to_patch[t_v].is_invalid())
            {
                auto r_v = _patch.vertices().add();
                _patch_pos[r_v] = _em.target_pos()[t_v];
                _v_target_to_patch[t_v] = r_v;
            }
        }

        // Add face to result mesh
        _patch.faces().add(t_f.vertices().to_vector([&] (auto t_v) {
            return _v_target_to_patch[t_v];
        }));

        // Fill halfedge index map
        for (auto t_h : t_f.halfedges())
        {
            const auto r_v_from = _v_target_to_patch[t_h.vertex_from()];
            const auto r_v_to = _v_target_to_patch[t_h.vertex_to()];
            const auto r_h = pm::halfedge_from_to(r_v_from, r_v_to);
            LE_ASSERT(r_h.is_valid());

            _h_patch_to_target[r_h] = t_h;
            _h_patch_to_target[r_h.opposite()] = t_h.opposite();
        }
    }
}

}

pm::edge_attribute<int> choose_loop_subdivisions(
        const Embedding& _em,
        const double _target_edge_length)
{
    // Assert embedded quad layout
    LE_ASSERT(_em.is_complete());
    for (auto f : _em.layout_mesh().faces())
        LE_ASSERT(f.vertices().size() == 4);

    auto subdivisions = _em.layout_mesh().edges().make_attribute<int>(0);

    // Iterate over loops
    auto visited = _em.layout_mesh().edges().make_attribute<bool>(false);
    for (auto l_e : _em.layout_mesh().edges())
    {
        if (visited[l_e])
            continue;

        // Trace loop and sum objective function
        auto calc_loop_objective = [&] (auto h_seed, int subdiv_add)
        {
            auto h = h_seed;
            auto obj = 0.0;
            do
            {
                const double length_sub = _em.embedded_path_length(h) / (subdivisions[h.edge()] + subdiv_add + 1);
                obj += pow(length_sub - _target_edge_length, 2);

                visited[h.edge()] = true;
                h = h.next().next().opposite();
            }
            while (h != h_seed);
            return obj;
        };

        // Increase subdivision as long as it reduces objective
        while (calc_loop_objective(l_e.halfedgeA(), 1) < calc_loop_objective(l_e.halfedgeA(), 0))
        {
            auto h = l_e.halfedgeA();
            do
            {
                subdivisions[h.edge()] += 1;
                h = h.next().next().opposite();
            }
            while (h != l_e.halfedgeA());
        }
    }

    return subdivisions;
}

pm::halfedge_attribute<tg::dpos2> parametrize_patches(
        const Embedding& _em,
        const pm::edge_attribute<int>& _l_subdivisions)
{
    LE_ASSERT(_em.is_complete());
    auto param = _em.target_mesh().halfedges().make_attribute<tg::dpos2>();

    for (auto l_f : _em.layout_mesh().faces())
    {
        LE_ASSERT(l_f.vertices().size() == 4);

        // Extract patch mesh
        pm::Mesh p_m;
        pm::vertex_attribute<tg::pos3> p_pos;
        pm::vertex_attribute<pm::vertex_handle> v_target_to_patch;
        pm::halfedge_attribute<pm::halfedge_handle> h_patch_to_target;
        extract_patch(_em, l_f, p_m, p_pos, v_target_to_patch, h_patch_to_target);

        // Constrain patch boundary to rectangle
        auto p_constrained = p_m.vertices().make_attribute<bool>(false);
        auto p_constraint_value = p_m.vertices().make_attribute<tg::dpos2>();

        const double texture_scale = 0.5;
        const double width = texture_scale * (_l_subdivisions[l_f.halfedges().first().edge()] + 1.0);
        const double height = texture_scale * (_l_subdivisions[l_f.halfedges().last().edge()] + 1.0);
        const std::vector<tg::dpos2> corners = { {0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height} };
        int corner_idx = 0;
        for (auto l_h : l_f.halfedges())
        {
            const double length_total = _em.embedded_path_length(l_h);
            double length_acc = 0.0;
            const auto t_path_vertices = _em.get_embedded_path(l_h);
            for (int i = 0; i < t_path_vertices.size() - 1; ++i)
            {
                const auto t_vi = t_path_vertices[i];
                const auto t_vj = t_path_vertices[i+1];
                const double lambda_i = length_acc / length_total;
                length_acc += tg::length(_em.target_pos()[t_vi] - _em.target_pos()[t_vj]);

                const auto p_vi = v_target_to_patch[t_vi];
                p_constrained[p_vi] = true;
                p_constraint_value[p_vi] = (1.0 - lambda_i) * corners[corner_idx] + lambda_i * corners[(corner_idx + 1) % 4];
            }

            ++corner_idx;
        }

        // Compute Tutte embedding
        pm::vertex_attribute<tg::dpos2> p_param;
        harmonic(p_pos, p_constrained, p_constraint_value, p_param);

        // Transfer parametrization to target mesh
        for (auto p_h : p_m.halfedges())
        {
            if (!p_h.is_boundary())
                param[h_patch_to_target[p_h]] = p_param[p_h.vertex_to()];
        }
    }

    return param;
}

}
