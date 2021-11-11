#include "QuadMeshing.hh"

#include <LayoutEmbedding/Harmonic.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/ExactPredicates.h>
#include <LayoutEmbedding/Util/Assert.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

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
        const double _target_edge_length,
        const int _max)
{
    // Assert embedded quad layout
    LE_ASSERT(_em.is_complete());
    for (auto f : _em.layout_mesh().faces())
        LE_ASSERT_EQ(f.vertices().size(), 4);

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

    subdivisions = subdivisions.map([&] (int s) { return std::min(_max, s); });

    return subdivisions;
}

HalfedgeParam parametrize_patches(
        const Embedding& _em,
        const pm::edge_attribute<int>& _l_subdivisions)
{
    LE_ASSERT(_em.is_complete());
    auto param = _em.target_mesh().halfedges().make_attribute<tg::dpos2>();

    // Ensure that _l_subdivisions are loop-wise consistent
    for (auto l_h : _em.layout_mesh().halfedges())
    {
        auto l_e = l_h.edge();
        auto l_e_opp = l_h.next().next().edge();
        LE_ASSERT_EQ(_l_subdivisions[l_e], _l_subdivisions[l_e_opp]);
    }

    for (auto l_f : _em.layout_mesh().faces())
    {
        LE_ASSERT_EQ(l_f.vertices().size(), 4);

        // Extract patch mesh
        pm::Mesh p_m;
        pm::vertex_attribute<tg::pos3> p_pos;
        pm::vertex_attribute<pm::vertex_handle> v_target_to_patch;
        pm::halfedge_attribute<pm::halfedge_handle> h_patch_to_target;
        extract_patch(_em, l_f, p_m, p_pos, v_target_to_patch, h_patch_to_target);

        // Constrain patch boundary to rectangle
        auto p_constrained = p_m.vertices().make_attribute<bool>(false);
        auto p_constraint_value = p_m.vertices().make_attribute<tg::dpos2>();

        const double width = _l_subdivisions[l_f.halfedges().first().edge()] + 1.0;
        const double height = _l_subdivisions[l_f.halfedges().last().edge()] + 1.0;
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
        // Try a few times with successively more uniform weights
        VertexParam p_param;
        if (!harmonic_parametrization(p_pos, p_constrained, p_constraint_value, p_param, LaplaceWeights::MeanValue, false))
        {
            if (!harmonic_parametrization(p_pos, p_constrained, p_constraint_value, p_param, LaplaceWeights::Uniform, true))
            {
                LE_ERROR_THROW("Harmonic parametrization failed.");
            }
        }

        for (auto v : p_m.vertices())
        {
            LE_ASSERT(std::isfinite(p_param[v].x));
            LE_ASSERT(std::isfinite(p_param[v].y));
        }

        // Transfer parametrization to target mesh
        for (auto p_h : p_m.halfedges())
        {
            if (!p_h.is_boundary())
                param[h_patch_to_target[p_h]] = p_param[p_h.vertex_to()];
        }
    }

    return param;
}

namespace
{

tg::ipos2 snap(
        const tg::dpos2& uv)
{
    const auto res = tg::ipos2(lround(uv.x), lround(uv.y));
    LE_ASSERT_EPS(res.x, uv.x, 1e-6);
    LE_ASSERT_EPS(res.y, uv.y, 1e-6);

    return res;
}

int count_subdiv(
        const Embedding& _em,
        const pm::halfedge_handle& l_h,
        const HalfedgeParam& _param)
{
    const auto path = _em.get_embedded_path(l_h);
    const auto h_first = pm::halfedge_from_to(path[0], path[1]);
    const auto h_last = pm::halfedge_from_to(path[path.size() - 2], path[path.size() - 1]);
    LE_ASSERT(h_first.is_valid());
    LE_ASSERT(h_last.is_valid());

    const auto uv_from = snap(_param[h_first.prev()]);
    const auto uv_to = snap(_param[h_last]);
    if (uv_from.x == uv_to.x)
        return abs(uv_to.y - uv_from.y) - 1;
    else if (uv_from.y == uv_to.y)
        return abs(uv_to.x - uv_from.x) - 1;
    else
        LE_ERROR_THROW("");
}

const double* ptr(
        const tg::dpos2& _p)
{
    return &_p.x;
}

bool in_triangle_inclusive(
        const tg::dpos2& _p,
        tg::dpos2 _a, tg::dpos2 _b, tg::dpos2 _c,
        const double _scale = 1.0)
{
    if (_scale != 1.0)
    {
        const auto cog = tg::average(std::vector<tg::dpos2> { _a, _b, _c });
        const auto M = tg::translation(cog) * tg::scaling(_scale, _scale) * tg::translation(-cog);
        _a = M * _a;
        _b = M * _b;
        _c = M * _c;
    }

    return orient2d(ptr(_p), ptr(_a), ptr(_b)) >= 0 &&
           orient2d(ptr(_p), ptr(_b), ptr(_c)) >= 0 &&
           orient2d(ptr(_p), ptr(_c), ptr(_a)) >= 0;
}

std::pair<double, double> compute_bary(
        const tg::dpos2& _p,
        const tg::dpos2& _a, const tg::dpos2& _b, const tg::dpos2& _c)
{
    const auto va = _a - _c;
    const auto vb = _b - _c;
    const auto vp = _p - _c;

    const double d00 = tg::dot(va, va);
    const double d01 = tg::dot(va, vb);
    const double d02 = tg::dot(va, vp);
    const double d11 = tg::dot(vb, vb);
    const double d12 = tg::dot(vb, vp);

    const double denom = d00 * d11 - d01 * d01;

    const double alpha = (d02 * d11 - d01 * d12) / denom;
    const double beta = (d00 * d12 - d01 * d02) / denom;

    return std::make_pair(alpha, beta);
}

tg::pos3 point_on_surface(
        const tg::dpos2& _p,
        const std::vector<pm::face_handle> _t_patch,
        const pm::vertex_attribute<tg::pos3> _pos,
        const HalfedgeParam& _param)
{
    // To fix numerical issues at the patch boundary,
    // try the lookup a few times while slowly growing each individual triangle.
    const int n_attempts = 3;
    double scale = 1.0;
    const double eps = 1e-6;
    for (int i = 0; i < n_attempts; ++i)
    {
        for (auto t_f : _t_patch)
        {
            LE_ASSERT_EQ(t_f.halfedges().size(), 3);
            const auto ha = t_f.halfedges().first(); // pointing to vertex a
            const auto hb = ha.next(); // pointing to vertex b
            const auto hc = hb.next(); // pointing to vertex c

            if (in_triangle_inclusive(_p, _param[ha], _param[hb], _param[hc], scale))
            {
                auto [alpha, beta] = compute_bary(_p, _param[ha], _param[hb], _param[hc]);
                if (!std::isfinite(alpha) || !std::isfinite(beta))
                {
                    alpha = 1.0 / 3.0;
                    beta = 1.0 / 3.0;
//                    std::cout << "Computing barycentric coordinates failed due to degenerate triangle." << std::endl;
                }
                return alpha * _pos[ha.vertex_to()] + beta * _pos[hb.vertex_to()] + (1.0 - alpha - beta) * _pos[hc.vertex_to()];
            }
        }

        scale *= 1.0 + eps;
    }

    LE_ERROR_THROW("Triangle lookup failed");
}

}

pm::vertex_attribute<tg::pos3> extract_quad_mesh(
        const Embedding& _em,
        const HalfedgeParam& _param,
        pm::Mesh& _q,
        pm::face_attribute<pm::face_handle>& _q_matching_layout_face)
{
    exactinit();

    _q.clear();
    auto q_pos = _q.vertices().make_attribute<tg::pos3>();
    _q_matching_layout_face = _q.faces().make_attribute<pm::face_handle>();

    // Per layout vertex, cache vertex index.
    // Per layout halfedge, cache list of vertex indices. First and last stay unused.
    auto vv_cache = _em.layout_mesh().vertices().make_attribute<pm::vertex_handle>();
    auto hv_cache = _em.layout_mesh().halfedges().make_attribute<std::vector<pm::vertex_handle>>();

    for (auto l_f : _em.layout_mesh().faces())
    {
        // Determine patch dimensions
        const auto l_h_u = l_f.halfedges().first(); // u direction
        const auto l_h_v = l_h_u.next();
        const int n_u = count_subdiv(_em, l_h_u, _param) + 2;
        const int n_v = count_subdiv(_em, l_h_v, _param) + 2;

        // Per layout face, cache grid of vertex indices.
        // First halfedge defines u direction.
        std::vector<std::vector<pm::vertex_handle>> fv_cache(n_u, std::vector<pm::vertex_handle>(n_v));

        // Get patch target triangles
        const auto t_patch = _em.get_patch(l_f);
        LE_ASSERT(!t_patch.empty());

        // Enumerate patch vertices
        for (int u = 0; u < n_u; ++u)
        {
            for (int v = 0; v < n_v; ++v)
            {
                // Look-up vertex in cache
                pm::vertex_handle q_v;
                if ((u == 0 || u == n_u - 1) && (v == 0 || v == n_v - 1))
                {
                    // Patch vertex
                    pm::vertex_handle l_v;
                    if (u == 0 && v == 0)
                        l_v = l_h_u.vertex_from();
                    else if (u == n_u - 1 && v == 0)
                        l_v = l_h_u.vertex_to();
                    else if (u == n_u - 1 && v == n_v - 1)
                        l_v = l_h_v.vertex_to();
                    else if (u == 0 && v == n_v - 1)
                        l_v = l_h_v.next().vertex_to();
                    else
                        LE_ERROR_THROW("");

                    // Cache lookup
                    q_v = vv_cache[l_v];
                    if (q_v.is_invalid())
                    {
                        q_v = _q.vertices().add();
                        vv_cache[l_v] = q_v;
                    }
                }
                else if (u == 0 || u == n_u - 1 || v == 0 || v == n_v - 1)
                {
                    // Patch boundary interior vertex
                    pm::halfedge_handle l_h;
                    int idx = -1;
                    int n = -1;
                    if (v == 0)
                    {
                        l_h = l_h_u;
                        idx = u;
                        n = n_u;
                    }
                    else if (u == n_u - 1)
                    {
                        l_h = l_h_v;
                        idx = v;
                        n = n_v;
                    }
                    else if (v == n_v - 1)
                    {
                        l_h = l_h_v.next();
                        idx = n_u - 1 - u;
                        n = n_u;
                    }
                    else if (u == 0)
                    {
                        l_h = l_h_u.prev();
                        idx = n_v - 1 - v;
                        n = n_v;
                    }
                    else
                        LE_ERROR_THROW("");
                    LE_ASSERT_GEQ(idx, 1);
                    LE_ASSERT_L(idx, n - 1);

                    // Init halfedge cache vectors
                    if (hv_cache[l_h].empty())
                        hv_cache[l_h] = std::vector<pm::vertex_handle>(n);
                    const auto l_h_opp = l_h.opposite();
                    if (hv_cache[l_h_opp].empty())
                        hv_cache[l_h_opp] = std::vector<pm::vertex_handle>(n);

                    LE_ASSERT_EQ(hv_cache[l_h].size(), n);
                    LE_ASSERT_EQ(hv_cache[l_h_opp].size(), n);

                    // Cache lookup
                    LE_ASSERT(hv_cache[l_h][idx] == hv_cache[l_h_opp][n - 1 - idx]);
                    q_v = hv_cache[l_h][idx];
                    if (q_v.is_invalid())
                    {
                        q_v = _q.vertices().add();
                        hv_cache[l_h][idx] = q_v;
                        hv_cache[l_h_opp][n - 1 - idx] = q_v;
                    }
                }
                else
                {
                    // Patch interior vertex
                    q_v = _q.vertices().add();
                }

                // Compute position
                const auto p_param = tg::dpos2((double)u, (double)v);
                q_pos[q_v] = point_on_surface(p_param, t_patch, _em.target_pos(), _param);

                // Add vertex to cache
                fv_cache[u][v] = q_v;

                // Add face
                if (u >= 1 && v >= 1)
                {
                    const auto q_f = _q.faces().add(
                            fv_cache[u-1][v-1],
                            fv_cache[u][v-1],
                            fv_cache[u][v],
                            fv_cache[u-1][v]);

                    _q_matching_layout_face[q_f] = l_f;
                }
            }
        }
    }

    // Assert no boundaries
    for (auto e : _q.edges())
        LE_ASSERT(!e.is_boundary());

    for (auto v : _q.vertices())
    {
        LE_ASSERT(std::isfinite(q_pos[v].x));
        LE_ASSERT(std::isfinite(q_pos[v].y));
        LE_ASSERT(std::isfinite(q_pos[v].z));
    }

    return q_pos;
}

}
