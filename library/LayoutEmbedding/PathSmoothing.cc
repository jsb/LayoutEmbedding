#include "PathSmoothing.hh"

#include <LayoutEmbedding/Snake.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Harmonic.hh>
#include <glow-extras/timing/CpuTimer.hh>
#include <queue>

namespace LayoutEmbedding
{

namespace
{

void extract_flap_region(
        const Embedding& _em,
        const pm::halfedge_handle& _l_h,
        pm::Mesh& _region,
        pm::vertex_attribute<tg::pos3>& _region_pos,
        pm::vertex_attribute<pm::vertex_handle>& _v_target_to_region,
        pm::halfedge_attribute<pm::halfedge_handle>& _h_region_to_target)
{
    // Init result
    _region.clear();
    _region_pos = _region.vertices().make_attribute<tg::pos3>();

    // Index maps
    _v_target_to_region = _em.target_mesh().vertices().make_attribute<pm::vertex_handle>();
    _h_region_to_target = _region.halfedges().make_attribute<pm::halfedge_handle>();

    // Get target faces inside flap
    const auto patch_A = _em.get_patch(_l_h.face());
    const auto patch_B = _em.get_patch(_l_h.opposite_face());
    auto flap = patch_A;
    flap.insert(flap.end(), patch_B.begin(), patch_B.end());

    // Create region mesh
    for (auto t_f : flap)
    {
        // Add vertices to result mesh
        for (auto t_v : t_f.vertices())
        {
            if (_v_target_to_region[t_v].is_invalid())
            {
                auto r_v = _region.vertices().add();
                _region_pos[r_v] = _em.target_pos()[t_v];
                _v_target_to_region[t_v] = r_v;
            }
        }

        // Add face to result mesh
        _region.faces().add(t_f.vertices().to_vector([&] (auto t_v) {
            return _v_target_to_region[t_v];
        }));

        // Fill halfedge index map
        for (auto t_h : t_f.halfedges())
        {
            const auto r_v_from = _v_target_to_region[t_h.vertex_from()];
            const auto r_v_to = _v_target_to_region[t_h.vertex_to()];
            const auto r_h = pm::halfedge_from_to(r_v_from, r_v_to);
            LE_ASSERT(r_h.is_valid());

            _h_region_to_target[r_h] = t_h;
            _h_region_to_target[r_h.opposite()] = t_h.opposite();
        }
    }
}

template <typename MatrixT, typename VectorT>
void append_as_row(
        MatrixT& _M,
        const VectorT& _v) // column vector to be appended as row
{
    _M.conservativeResize(_M.rows() + 1, _M.cols());
    _M.row(_M.rows() - 1) = _v.transpose();
}

void flap_n_gon_boundary(
        const Embedding& _em,
        const pm::halfedge_handle& _l_h,
        const pm::vertex_attribute<pm::vertex_handle>& _v_target_to_region,
        const pm::Mesh& _region,
        pm::vertex_attribute<bool>& _constrained,
        pm::vertex_attribute<tg::dpos2>& _constraint_pos)
{
    _constrained = _region.vertices().make_attribute<bool>(false);
    _constraint_pos = _region.vertices().make_attribute<tg::dpos2>(tg::dpos2(0.0, 0.0));

    // Collect inner halfedges of flap boundary
    std::vector<pm::halfedge_handle> l_hs_boundary;
    {
        auto h = _l_h.next();
        while (h != _l_h)
        {
            l_hs_boundary.push_back(h);
            h = h.next();
        }
        h = _l_h.opposite().next();
        while (h != _l_h.opposite())
        {
            l_hs_boundary.push_back(h);
            h = h.next();
        }
    }

    double boundary_length_total = 0.0;
    for (auto l_h : l_hs_boundary)
         boundary_length_total += _em.embedded_path_length(l_h);

    double boundary_length_acc = 0.0;
    for (auto l_h : l_hs_boundary)
    {
        const auto t_path = _em.get_embedded_path(l_h);
        const double path_length_total = _em.embedded_path_length(l_h);

        // Position one-ring layout vertices on unit circle
        auto angle_from = boundary_length_acc / boundary_length_total * 2.0 * M_PI;
        boundary_length_acc += path_length_total;
        auto angle_to = boundary_length_acc / boundary_length_total * 2.0 * M_PI;
        LE_ASSERT(angle_from >= 0.0);
        LE_ASSERT(angle_from < 2.0 * M_PI);
        LE_ASSERT(angle_to >= 0.0);
        LE_ASSERT(angle_to <= 2.0 * M_PI);
        LE_ASSERT(angle_from < angle_to);
        auto p_from = tg::dpos2(cos(angle_from), sin(angle_from));
        auto p_to = tg::dpos2(cos(angle_to), sin(angle_to));

        // Position all other boundary vertices on straight lines
        LE_ASSERT(t_path.front() == _em.matching_target_vertex(l_h.vertex_from()));
        LE_ASSERT(t_path.back() == _em.matching_target_vertex(l_h.vertex_to()));
        double path_length_acc = 0.0;
        for (int i = 0; i < t_path.size() - 1; ++i) // Last vertex is handled by next path
        {
            const double lambda = path_length_acc / path_length_total;
            const auto r_v = _v_target_to_region[t_path[i]];
            _constrained[r_v] = true;
            _constraint_pos[r_v] = (1.0 - lambda) * p_from + lambda * p_to;

            path_length_acc += tg::length(_em.target_pos()[t_path[i + 1]] - _em.target_pos()[t_path[i]]);
        }
    }
}

Snake transfer_snake_to_target(
        const Snake& _snake,
        const pm::halfedge_attribute<pm::halfedge_handle>& _h_map)
{
    Snake res = _snake;

    for (auto& v : res.vertices)
        v.h = _h_map[v.h];

    return res;
}

/**
 * Parametrize flap and straighten edge.
 */
bool smooth_path(
        Embedding& _em,
        const pm::halfedge_handle& _l_h)
{
    // Extract flap region mesh
    pm::Mesh region;
    pm::vertex_attribute<tg::pos3> region_pos;
    pm::vertex_attribute<pm::vertex_handle> v_target_to_region;
    pm::halfedge_attribute<pm::halfedge_handle> h_region_to_target;
    extract_flap_region(_em, _l_h, region, region_pos, v_target_to_region, h_region_to_target);

    // TODO FIXME: Split edges with two adjacent boundary edges on same path

    // Construct 2D n-gon
    pm::vertex_attribute<bool> constrained;
    pm::vertex_attribute<tg::dpos2> constraint_pos;
    flap_n_gon_boundary(_em, _l_h, v_target_to_region, region, constrained, constraint_pos);

    // Compute harmonic parametrization
    pm::vertex_attribute<tg::dpos2> region_param;
    if (!harmonic(region_pos, constrained, constraint_pos, region_param))
    {
        std::cout << "WARNING: Failed to smooth a path." << std::endl;
        return false;
    }

    // Compute snake by tracing straight line in parametrization
    const auto r_v_from = v_target_to_region[_em.matching_target_vertex(_l_h.vertex_from())];
    const auto r_v_to = v_target_to_region[_em.matching_target_vertex(_l_h.vertex_to())];
    const auto r_snake = snake_from_parametrization(region_param, r_v_from, r_v_to);
    const auto t_snake = transfer_snake_to_target(r_snake, h_region_to_target);

    // Embed snake in target mesh
    _em.unembed_path(_l_h);
    _em.embed_path(_l_h, t_snake);

    return true;
}

}

Embedding smooth_paths(
        const Embedding& _em_orig,
        const int _n_iters)
{
    glow::timing::CpuTimer timer;

    Embedding em = _em_orig; // copy

    for (int iter = 0; iter < _n_iters; ++iter)
    {
        for (auto l_e : em.layout_mesh().edges())
            smooth_path(em, l_e.halfedgeA());
    }

    std::cout << "Smoothing paths (" << _n_iters << " iterations ) took "
              << timer.elapsedSecondsD() << " s. "
              << "Resulting mesh has " << em.target_mesh().vertices().size() << " vertices."
              << std::endl;

    return em;
}

}
