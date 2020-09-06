#include "Harmonic.hh"

#include <LayoutEmbedding/Assert.hh>
#include <Eigen/SparseLU>

namespace LayoutEmbedding
{

namespace
{

/// Angle at to-vertex between given and next halfedge
auto calc_sector_angle(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::halfedge_handle& _h)
{
    const auto v1 = _pos[_h.next().vertex_to()] - _pos[_h.vertex_to()];
    const auto v2 = _pos[_h.vertex_from()] - _pos[_h.vertex_to()];
    return tg::angle_between(v1, v2);
}

double mean_value_weight(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::halfedge_handle& _h)
{
    if (_h.edge().is_boundary())
        return 0.0;

    const auto angle_l = calc_sector_angle(_pos, _h.prev());
    const auto angle_r = calc_sector_angle(_pos, _h.opposite());
    const auto edge_length = pm::edge_length(_h, _pos);
    double w_ij = (tan(angle_l.radians() / 2.0) + tan(angle_r.radians() / 2.0)) / edge_length;

    if (w_ij <= 0.0)
        w_ij = 1e-5;

    return w_ij;
}

}

bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const pm::vertex_attribute<Eigen::VectorXd>& _constraint_value,
        Eigen::MatrixXd& _res)
{
    LE_ASSERT(_pos.mesh().is_compact());

    const int n = _pos.mesh().vertices().size();
    const int d = _constraint_value.first().size();

    // Set up Laplace matrix and rhs
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n, d);
    std::vector<Eigen::Triplet<double>> triplets;
    for (auto v : _pos.mesh().vertices())
    {
        const int i = v.idx.value;

        if (_constrained[v])
        {
            LE_ASSERT(_constraint_value[v].size() == d);

            triplets.push_back(Eigen::Triplet<double>(i, i, 1.0));
            rhs(i, 0) = _constraint_value[v][0];
            rhs(i, 1) = _constraint_value[v][1];
        }
        else
        {
            LE_ASSERT(!v.is_boundary());

            for (auto h : v.outgoing_halfedges())
            {
                const int j = h.vertex_to().idx.value;
                const double w_ij = mean_value_weight(_pos, h);
                triplets.push_back(Eigen::Triplet<double>(i, j, w_ij));
                triplets.push_back(Eigen::Triplet<double>(i, i, -w_ij));
            }
        }
    }

    Eigen::SparseMatrix<double> L(n, n);
    L.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(L);
    if (solver.info() != Eigen::Success)
        return false;

    _res = solver.solve(rhs);
    if (solver.info() != Eigen::Success)
        return false;

    return true;;
}

bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const pm::vertex_attribute<tg::dpos2>& _constraint_value,
        pm::vertex_attribute<tg::dpos2>& _res)
{
    auto to_eigen = [] (auto p) { Eigen::VectorXd v(2); v[0] = p[0]; v[1] = p[1]; return v; };
    Eigen::MatrixXd res_mat;
    if (!harmonic(_pos, _constrained, _constraint_value.map(to_eigen), res_mat))
        return false;

    _res = _pos.mesh().vertices().make_attribute<tg::dpos2>();
    for (auto v : _pos.mesh().vertices())
        _res[v] = tg::dpos2(res_mat(v.idx.value, 0), res_mat(v.idx.value, 1));

    return true;
}

}
