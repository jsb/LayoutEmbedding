#pragma once

#include <Eigen/Dense>
#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>
#include <LayoutEmbedding/Parametrization.hh>

namespace LayoutEmbedding
{

enum class LaplaceWeights
{
    Uniform,
    MeanValue,
};

/// Compute harmonic field using mean-value weights.
bool harmonic(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const Eigen::MatrixXd& _constraint_values,
        Eigen::MatrixXd& _res,
        const LaplaceWeights _weights,
        const bool _fallback_iterative = false);

/// Compute harmonic field using mean-value weights.
bool harmonic_parametrization(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const pm::vertex_attribute<bool>& _constrained,
        const VertexParam& _constraint_values,
        VertexParam& _res,
        const LaplaceWeights _weights,
        const bool _fallback_iterative = false);

}
