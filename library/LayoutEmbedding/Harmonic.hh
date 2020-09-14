#pragma once

#include <Eigen/Dense>
#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

namespace LayoutEmbedding
{

using Parametrization = pm::vertex_attribute<tg::dpos2>;

// Compute harmonic field using mean-value weights.
bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const Eigen::MatrixXd& _constraint_values,
        Eigen::MatrixXd& _res,
        const double _lambda_uniform = 0.0);

bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const Parametrization& _constraint_values,
        Parametrization& _res,
        const double _lambda_uniform = 0.0);

bool injective(
        const Parametrization& _param);

}
