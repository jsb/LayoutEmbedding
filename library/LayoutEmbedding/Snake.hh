#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg-lean.hh>
#include <Eigen/Dense>

namespace LayoutEmbedding
{

struct SnakeVertex
{
    template <typename VectorT>
    VectorT point(
            const pm::vertex_attribute<VectorT>& _pos) const
    {
        return (1.0 - lambda) * _pos[h.vertex_from()] + lambda * _pos[h.vertex_to()];
    };

    pm::halfedge_handle h;
    double lambda;
};

struct Snake
{
    std::vector<SnakeVertex> vertices;
};

/**
 * Compute a snake by intersecting a straight line
 * segment with a mesh parametrization in the plane.
 * _param: |V| x 2 matrix.
 */
Snake snake_from_parametrization(
        const Eigen::MatrixXd& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to);

}
