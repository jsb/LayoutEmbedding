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
        const Eigen::MatrixXd& _constraint_values,
        Eigen::MatrixXd& _res,
        const LaplaceWeights _weights,
        const bool _fallback_iterative)
{
    LE_ASSERT(_pos.mesh().is_compact());

    const int n = _pos.mesh().vertices().size();
    const int d = _constraint_values.cols();
    LE_ASSERT(_constraint_values.rows() == n);

    // Set up Laplace matrix and rhs
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n, d);
    std::vector<Eigen::Triplet<double>> triplets;
    for (auto v : _pos.mesh().vertices())
    {
        const int i = v.idx.value;

        if (_constrained[v])
        {
            triplets.push_back(Eigen::Triplet<double>(i, i, 1.0));
            rhs.row(i) = _constraint_values.row(i);
        }
        else
        {
            LE_ASSERT(!v.is_boundary());

            for (auto h : v.outgoing_halfedges())
            {
                const int j = h.vertex_to().idx.value;
                double w_ij;
                if (_weights == LaplaceWeights::Uniform)
                    w_ij = 1.0;
                else if (_weights == LaplaceWeights::MeanValue)
                    w_ij = mean_value_weight(_pos, h);
                else
                    LE_ERROR_THROW("");

                triplets.push_back(Eigen::Triplet<double>(i, j, w_ij));
                triplets.push_back(Eigen::Triplet<double>(i, i, -w_ij));
            }
        }
    }

    Eigen::SparseMatrix<double> L(n, n);
    L.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(L);
    if (solver.info() == Eigen::Success)
    {
        _res = solver.solve(rhs);
        if (solver.info() == Eigen::Success)
            return true;
    }

    std::cout << "LU solve failed" << std::endl;

    if (_fallback_iterative)
    {
        std::cout << "Falling back to iterative solver" << std::endl;
    }

    return false;
}

bool harmonic_parametrization(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const VertexParam& _constraint_values,
        VertexParam& _res,
        const LaplaceWeights _weights,
        const bool _fallback_iterative)
{
    const int n = _pos.mesh().vertices().size();
    const int d = 2;

    // Convert constraints
    Eigen::MatrixXd constraint_values = Eigen::MatrixXd::Zero(n, d);
    for (auto v : _pos.mesh().vertices())
        constraint_values.row(v.idx.value) = Eigen::Vector2d(_constraint_values[v].x, _constraint_values[v].y);

    // Compute
    Eigen::MatrixXd res_mat;
    if (!harmonic(_pos, _constrained, constraint_values, res_mat, _weights, _fallback_iterative))
        return false;

    // Convert result
    _res = _pos.mesh().vertices().make_attribute<tg::dpos2>();
    for (auto v : _pos.mesh().vertices())
        _res[v] = tg::dpos2(res_mat(v.idx.value, 0), res_mat(v.idx.value, 1));

    return true;
}

}
