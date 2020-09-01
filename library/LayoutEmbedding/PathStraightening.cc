#include "PathStraightening.hh"

#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/IGLMesh.hh>
#include <igl/harmonic.h>
#include <igl/intrinsic_delaunay_cotmatrix.h>
#include <queue>

namespace LayoutEmbedding
{

namespace
{

void extract_one_ring_region(
        const Embedding& _em,
        const pm::vertex_handle& _l_v,
        pm::Mesh& _region,
        pm::vertex_attribute<tg::pos3>& _region_pos,
        pm::vertex_attribute<pm::vertex_handle>& _v_target_to_region)
{
    // Init result
    _region.clear();
    _region_pos = _region.vertices().make_attribute<tg::pos3>();

    // Index map
    _v_target_to_region = _em.target_mesh().vertices().make_attribute<pm::vertex_handle>();

    // Region flood fill
    std::queue<pm::halfedge_handle> queue;
    for (auto l_h : _l_v.outgoing_halfedges())
        queue.push(_em.get_embedded_target_halfedge(l_h));

    auto visited = _em.target_mesh().faces().make_attribute<bool>(false);
    while (!queue.empty())
    {
        const auto t_h = queue.front();
        const auto t_f = t_h.face();
        queue.pop();

        // Already visited?
        if (visited[t_f])
            continue;
        visited[t_f] = true;

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

        // Enqueue neighbors (if not visited and not blocked)
        for (auto t_h_inside : t_f.halfedges())
        {
            const auto t_h_outside = t_h_inside.opposite();
            if (!visited[t_h_outside.face()] && !_em.is_blocked(t_h_outside.edge()))
                queue.push(t_h_outside);
        }
    }
}

template <typename MatrixT, typename VectorT>
void append_as_row(
        MatrixT& _M,
        const VectorT& _v) //column vector to be appended as row
{
    _M.conservativeResize(_M.rows() + 1, _M.cols());
    _M.row(_M.rows() - 1) = _v.transpose();
}

void n_gon_boundary(
        const Embedding& _em,
        const pm::vertex_handle& _l_v,
        const pm::vertex_attribute<pm::vertex_handle>& _v_target_to_region,
        Eigen::MatrixXi& _b, // boundary vertex indices
        Eigen::MatrixXd& _bc) // boundary coordinates as rows
{
    _b.resize(0, 1);
    _bc.resize(0, 2);

    double boundary_length_total = 0.0;
    for (auto l_h_out : _l_v.outgoing_halfedges())
         boundary_length_total += _em.embedded_path_length(l_h_out.next());

    double boundary_length_acc = 0.0;
    for (auto l_h_out : _l_v.outgoing_halfedges())
    {
        const auto l_h = l_h_out.next(); // ccw boundary he
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
        auto p_from = Eigen::Vector2d(cos(angle_from), sin(angle_from));
        auto p_to = Eigen::Vector2d(cos(angle_to), sin(angle_to));

        // Position all other boundary vertices on straight lines
        LE_ASSERT(t_path.front() == _em.matching_target_vertex(l_h.vertex_from()));
        LE_ASSERT(t_path.back() == _em.matching_target_vertex(l_h.vertex_to()));
        double path_length_acc = 0.0;
        for (int i = 0; i < t_path.size() - 1; ++i) // Last vertex is handled by next path
        {
            const double lambda = path_length_acc / path_length_total;
            append_as_row(_b, Eigen::Matrix<int, 1, 1>(_v_target_to_region[t_path[i]].idx.value));
            append_as_row(_bc, (1.0 - lambda) * p_from + lambda * p_to);

            path_length_acc += tg::length(_em.target_pos()[t_path[i + 1]] - _em.target_pos()[t_path[i]]);
        }
    }
}

Eigen::MatrixXd harmonic_param(
        const IGLMesh& _region_igl,
        const Eigen::MatrixXi& _b,
        const Eigen::MatrixXd& _bc)
{
    // Compute harmonic parametrization using intrinsic cotan weights
    Eigen::SparseMatrix<double> L;
    igl::intrinsic_delaunay_cotmatrix(_region_igl.V, _region_igl.F, L);

    Eigen::SparseMatrix<double> M;
    igl::massmatrix(_region_igl.V, _region_igl.F, igl::MASSMATRIX_TYPE_VORONOI, M);

    Eigen::MatrixXd param;
    igl::harmonic(L, M, _b, _bc, 1, param);

    return param;
}

void straighten_one_ring(
        Embedding& _em,
        const pm::vertex_handle& _l_v)
{
    // Extract one ring region mesh
    pm::Mesh region;
    pm::vertex_attribute<tg::pos3> region_pos;
    pm::vertex_attribute<pm::vertex_handle> v_target_to_region;
    extract_one_ring_region(_em, _l_v, region, region_pos, v_target_to_region);
    const auto region_igl = to_igl_mesh(region_pos);

    // TODO FIXME: Split edges with two adjacent boundary edges on same path

    // Construct 2D n-gon
    Eigen::MatrixXi b; // boundary vertex indices
    Eigen::MatrixXd bc; // boundary coordinates as rows
    n_gon_boundary(_em, _l_v, v_target_to_region, b, bc);

    // Compute harmonic parametrization using intrinsic cotan weights
    Eigen::MatrixXd param = harmonic_param(region_igl, b, bc);

    pm::Mesh param_mesh;
    pm::vertex_attribute<tg::pos3> param_pos;
    from_igl_mesh(IGLMesh { param, region_igl.F }, param_mesh, param_pos);

//    gv::view(gv::lines(param_pos));
//    gv::view(region_pos);
}

}

Embedding straighten_paths(
        const Embedding& _em_orig)
{
    Embedding em = _em_orig; // copy

    for (auto l_v : em.layout_mesh().vertices())
        straighten_one_ring(em, l_v);

    return em;
}

}
