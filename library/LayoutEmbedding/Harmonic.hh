#pragma once

#include <Eigen/Dense>
#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

namespace LayoutEmbedding
{

// Compute harmonic field using mean-value weights.
bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const pm::vertex_attribute<Eigen::VectorXd>& _constraint_value,
        Eigen::MatrixXd& _res);

bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const pm::vertex_attribute<tg::dpos2>& _constraint_value,
        pm::vertex_attribute<tg::dpos2>& _res);

}
