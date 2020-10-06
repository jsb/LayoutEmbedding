#include "PathSmoothing.hh"

#include <LayoutEmbedding/Snake.hh>
#include <LayoutEmbedding/Harmonic.hh>
#include <LayoutEmbedding/Util/Assert.hh>

#include <glow-extras/timing/CpuTimer.hh>
#include <queue>

namespace LayoutEmbedding
{

namespace
{

}

Embedding subdivide(
        const Embedding& _em_orig,
        const int _n_iters)
{
    Embedding em = _em_orig; // copy

    for (int iter = 0; iter < _n_iters; ++iter)
    {
        // Backup mesh
        pm::Mesh m_orig;
        m_orig.copy_from(em.target_mesh());

        // Delet all faces
        std::vector<std::vector<pm::vertex_handle>> f_vs_orig;
        for (auto f : em.target_mesh().faces())
        {
            f_vs_orig.push_back(f.vertices().to_vector());
            em.target_mesh().faces().remove(f);
        }

        // Split vertices at edge midpoints
        auto e_v_orig = m_orig.edges().make_attribute<pm::vertex_handle>();
        auto h_label_orig = m_orig.halfedges().make_attribute<pm::halfedge_handle>();
        for (auto e_orig : m_orig.edges())
        {
            const auto e_before = em.target_mesh()[e_orig.idx];
            h_label_orig[e_orig.halfedgeA()] = em.matching_layout_halfedge(e_before.halfedgeA());
            h_label_orig[e_orig.halfedgeB()] = em.matching_layout_halfedge(e_before.halfedgeB());

            const auto p = tg::mix(em.target_pos()[e_before.vertexA()], em.target_pos()[e_before.vertexB()], 0.5);
            const auto v = em.target_mesh().edges().split(e_before);
            e_v_orig[e_orig] = v;
            em.target_pos()[v] = p;
        }

        // Insert four new faces in each old face
        for (auto f_orig : m_orig.faces())
        {
            const auto hs_orig = f_orig.halfedges().to_vector();
            const auto v0 = em.target_mesh()[hs_orig[0].vertex_from().idx];
            const auto v1 = e_v_orig[hs_orig[0].edge()];
            const auto v2 = em.target_mesh()[hs_orig[1].vertex_from().idx];
            const auto v3 = e_v_orig[hs_orig[1].edge()];
            const auto v4 = em.target_mesh()[hs_orig[2].vertex_from().idx];
            const auto v5 = e_v_orig[hs_orig[2].edge()];

            em.target_mesh().faces().add(v0, v1, v5);
            em.target_mesh().faces().add(v1, v2, v3);
            em.target_mesh().faces().add(v3, v4, v5);
            em.target_mesh().faces().add(v1, v3, v5);

            const auto h01 = pm::halfedge_from_to(v0, v1);
            const auto h12 = pm::halfedge_from_to(v1, v2);
            const auto h23 = pm::halfedge_from_to(v2, v3);
            const auto h34 = pm::halfedge_from_to(v3, v4);
            const auto h45 = pm::halfedge_from_to(v4, v5);
            const auto h50 = pm::halfedge_from_to(v5, v0);

            em.matching_layout_halfedge(h01) = h_label_orig[hs_orig[0]];
            em.matching_layout_halfedge(h01.opposite()) = h_label_orig[hs_orig[0].opposite()];
            em.matching_layout_halfedge(h12) = h_label_orig[hs_orig[0]];
            em.matching_layout_halfedge(h12.opposite()) = h_label_orig[hs_orig[0].opposite()];
            em.matching_layout_halfedge(h23) = h_label_orig[hs_orig[1]];
            em.matching_layout_halfedge(h23.opposite()) = h_label_orig[hs_orig[1].opposite()];
            em.matching_layout_halfedge(h34) = h_label_orig[hs_orig[1]];
            em.matching_layout_halfedge(h34.opposite()) = h_label_orig[hs_orig[1].opposite()];
            em.matching_layout_halfedge(h45) = h_label_orig[hs_orig[2]];
            em.matching_layout_halfedge(h45.opposite()) = h_label_orig[hs_orig[2].opposite()];
            em.matching_layout_halfedge(h50) = h_label_orig[hs_orig[2]];
            em.matching_layout_halfedge(h50.opposite()) = h_label_orig[hs_orig[2].opposite()];
        }

        em.target_mesh().compactify();
    }

    return em;
}

namespace
{

void preprocess_split_edges(
        Embedding& _em)
{
    // Split non-boundary edges with both end vertices on the same path
    int n_splits = 0;
    const auto t_hes_orig = _em.target_mesh().halfedges();
    for (auto t_h : t_hes_orig)
    {
        if (_em.matching_layout_halfedge(t_h).is_invalid() &&
            _em.matching_layout_halfedge(t_h.next()).is_valid() &&
            _em.matching_layout_halfedge(t_h.prev()).is_valid())
        {
            const auto p = tg::mix(_em.target_pos()[t_h.vertex_from()], _em.target_pos()[t_h.vertex_to()], 0.5);
            const auto t_v = _em.target_mesh().edges().split_and_triangulate(t_h.edge());
            _em.target_pos()[t_v] = p;
            ++n_splits;
        }
    }

    if (n_splits > 0)
        std::cout << "Split " << n_splits << " edges during path smoothing preprocess." << std::endl;
}

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

void constrain_flap_boundary(
        const Embedding& _em,
        const pm::halfedge_handle& _l_h,
        const pm::vertex_attribute<pm::vertex_handle>& _v_target_to_region,
        const pm::Mesh& _region,
        pm::vertex_attribute<bool>& _constrained,
        pm::vertex_attribute<tg::dpos2>& _constraint_pos,
        const bool _quad_flap_to_rectangle)
{
    _constrained = _region.vertices().make_attribute<bool>(false);
    _constraint_pos = _region.vertices().make_attribute<tg::dpos2>(tg::dpos2(0.0, 0.0));

    // Collect sides of circle-inscribed n-gon
    std::vector<std::vector<pm::halfedge_handle>> l_sides;
    std::vector<std::vector<pm::vertex_handle>> t_sides;
    std::vector<double> side_lengths;
    double boundary_length_total = 0.0;
    auto add_side = [&] (const std::vector<pm::halfedge_handle>& lhs)
    {
        l_sides.push_back(lhs);
        t_sides.push_back(std::vector<pm::vertex_handle>());
        auto& t_side = t_sides.back();
        side_lengths.push_back(0.0);
        auto& length = side_lengths.back();

        for (auto lh : lhs)
        {
            const auto t_path = _em.get_embedded_path(lh);
            if (t_side.empty())
                t_side.insert(t_side.end(), t_path.begin(), t_path.end());
            else
            {
                LE_ASSERT(t_side.back() == t_path.front());
                t_side.insert(t_side.end(), t_path.begin() + 1, t_path.end());
            }
            length += _em.embedded_path_length(lh);
        }
        boundary_length_total += length;
    };

    if (_l_h.face().vertices().size() == 4 &&
        _l_h.opposite_face().vertices().size() == 4 &&
        _quad_flap_to_rectangle)
    {
        // Quad patch: merge flap to a single 4-gon
        auto lh = _l_h;
        lh = lh.prev();
        add_side({ lh, lh.next().opposite().next() });
        lh = lh.next().opposite().next().next();
        add_side({ lh });
        lh = lh.next();
        add_side({ lh, lh.next().opposite().next() });
        lh = lh.next().opposite().next().next();
        add_side({ lh });
    }
    else
    {
        // Non-quad patches: Constrain all flap vertices
        auto lh = _l_h.next();
        while (lh != _l_h)
        {
            add_side({ lh });
            lh = lh.next();
        }
        lh = _l_h.opposite().next();
        while (lh != _l_h.opposite())
        {
            add_side({ lh });
            lh = lh.next();
        }
    }

    // Constrain n-gon to circle-inscribed polygon.
    // Distribute target boundary vertices via unit-speed parametrization
    double boundary_length_acc = 0.0;
    for (int i_side = 0; i_side < l_sides.size(); ++i_side)
    {
        // Position one-ring layout vertices on unit circle
        auto angle_from = boundary_length_acc / boundary_length_total * 2.0 * M_PI;
        boundary_length_acc += side_lengths[i_side];
        auto angle_to = boundary_length_acc / boundary_length_total * 2.0 * M_PI;
        LE_ASSERT(angle_from >= 0.0);
        LE_ASSERT(angle_from < 2.0 * M_PI);
        LE_ASSERT(angle_to >= 0.0);
        LE_ASSERT(angle_to <= 2.0 * M_PI);
        LE_ASSERT(angle_from < angle_to);
        auto p_from = tg::dpos2(cos(angle_from), sin(angle_from));
        auto p_to = tg::dpos2(cos(angle_to), sin(angle_to));

        // Position target boundary vertices on straight line
        LE_ASSERT(t_sides[i_side].front() == _em.matching_target_vertex(l_sides[i_side].front().vertex_from()));
        LE_ASSERT(t_sides[i_side].back() == _em.matching_target_vertex(l_sides[i_side].back().vertex_to()));
        double side_length_acc = 0.0;
        for (int i_vertex = 0; i_vertex < t_sides[i_side].size() - 1; ++i_vertex) // Last vertex is handled by next path
        {
            const double lambda = side_length_acc / side_lengths[i_side];
            const auto r_v = _v_target_to_region[t_sides[i_side][i_vertex]];
            _constrained[r_v] = true;
            _constraint_pos[r_v] = (1.0 - lambda) * p_from + lambda * p_to;

            side_length_acc += tg::length(_em.target_pos()[t_sides[i_side][i_vertex + 1]] - _em.target_pos()[t_sides[i_side][i_vertex]]);
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
        const pm::halfedge_handle& _l_h,
        const bool _quad_flap_to_rectangle)
{
    // Extract flap region mesh
    pm::Mesh region;
    pm::vertex_attribute<tg::pos3> region_pos;
    pm::vertex_attribute<pm::vertex_handle> v_target_to_region;
    pm::halfedge_attribute<pm::halfedge_handle> h_region_to_target;
    extract_flap_region(_em, _l_h, region, region_pos, v_target_to_region, h_region_to_target);

    // Construct 2D n-gon
    pm::vertex_attribute<bool> constrained;
    VertexParam constraint_pos;
    constrain_flap_boundary(_em, _l_h, v_target_to_region, region, constrained, constraint_pos, _quad_flap_to_rectangle);

    // Compute harmonic parametrization
    // Try a few times with successively more uniform weights
    VertexParam region_param;
    if (!harmonic_parametrization(region_pos, constrained, constraint_pos, region_param, LaplaceWeights::MeanValue, false) || !injective(region_param))
    {
        if (!harmonic_parametrization(region_pos, constrained, constraint_pos, region_param, LaplaceWeights::Uniform, true) || !injective(region_param))
        {
            std::cout << "Path smoothing failed" << std::endl;
            return false;
        }
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
        const int _n_iters,
        const bool _quad_flap_to_rectangle)
{
    return smooth_paths(_em_orig, _em_orig.layout_mesh().edges().to_vector(), _n_iters, _quad_flap_to_rectangle);
}

Embedding smooth_paths(
        const Embedding& _em_orig,
        const std::vector<pm::edge_handle>& _l_edges,
        const int _n_iters,
        const bool _quad_flap_to_rectangle)
{
    glow::timing::CpuTimer timer;

    Embedding em = _em_orig; // copy

    // Split non-boundary edges with both end vertices on the same path
    preprocess_split_edges(em);

    for (int iter = 0; iter < _n_iters; ++iter)
    {
        for (auto l_e : _l_edges)
        {
            if (!l_e.is_boundary())
                smooth_path(em, l_e.halfedgeA(), _quad_flap_to_rectangle);
        }
    }

    std::cout << "Smoothing paths (" << _n_iters << " iterations ) took "
              << timer.elapsedSecondsD() << " s. "
              << "Resulting mesh has " << em.target_mesh().vertices().size() << " vertices."
              << std::endl;

    return em;
}

}
