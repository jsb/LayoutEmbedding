#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>
#include <Eigen/Dense>

namespace LayoutEmbedding
{

struct SnakeVertex
{
    template <typename PosT>
    PosT point(
            const pm::vertex_attribute<PosT>& _pos) const
    {
        return tg::mix(_pos[h.vertex_from()], _pos[h.vertex_to()], lambda);
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
 */
Snake snake_from_parametrization(
        const pm::vertex_attribute<tg::dpos2>& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to);

/**
 * Turn the Snake into a pure vertex path by splitting edges.
 * Returns embedded path as sequence of vertices.
 */
template <typename PosT>
std::vector<pm::vertex_handle> embed_snake(
        const Snake& _snake,
        pm::Mesh& _mesh,
        pm::vertex_attribute<PosT>& _pos);

}
