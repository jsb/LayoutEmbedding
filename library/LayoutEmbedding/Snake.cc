#include "Snake.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/ExactPredicates.h>

namespace LayoutEmbedding
{

namespace
{

/**
 * CALL exactinit() once before calling this function!
 * Does straight line segment (a, b) intersect (c, d)?
 * Inclusive: touching an endpoint counts as intersection.
 */
bool intersects_exact_inclusive_2d(
        const Eigen::Vector2d& a, const Eigen::Vector2d& b,
        const Eigen::Vector2d& c, const Eigen::Vector2d& d)
{
    const auto sign1 = orient2d(a.data(), b.data(), c.data());
    const auto sign2 = orient2d(a.data(), d.data(), b.data());
    const auto sign3 = orient2d(a.data(), d.data(), c.data());
    const auto sign4 = orient2d(b.data(), c.data(), d.data());

    int n_negative = 0;
    int n_positive = 0;
    if (sign1 <= 0) ++n_negative;
    if (sign1 >= 0) ++n_positive;
    if (sign2 <= 0) ++n_negative;
    if (sign2 >= 0) ++n_positive;
    if (sign3 <= 0) ++n_negative;
    if (sign3 >= 0) ++n_positive;
    if (sign4 <= 0) ++n_negative;
    if (sign4 >= 0) ++n_positive;

    if (n_negative == 4 || n_positive == 4)
        return true;

    return false;
}

/**
 * Intersects the line a to b with c to d and returns the
 * intersection parameter on the segment from a to b.
 * Does not perform any checks.
 */
double intersection_parameter(
        const Eigen::Vector2d& a, const Eigen::Vector2d& b,
        const Eigen::Vector2d& c, const Eigen::Vector2d& d)
{
    // Precondition: not collinear
    return ((a[0] - c[0]) * (c[1] - d[1]) - (a[1] - c[1]) * (c[0] - d[0])) /
           ((a[0] - b[0]) * (c[1] - d[1]) - (a[1] - b[1]) * (c[0] - d[0]));
}

/**
  * Performs intersection check of h with staight line segment.
  * If true: returns SnakeVertex with intersection parameter.
  */
std::optional<SnakeVertex> intersection_vertex(
        const pm::halfedge_handle& _h,
        const Eigen::MatrixXd& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to)
{
    auto p = [&_param] (auto v) { return _param.row(v.idx.value); };

    if (intersects_exact_inclusive_2d(
                p(_h.vertex_from()), p(_h.vertex_to()),
                p(_v_from), p(_v_to)))
    {
        const double lambda = intersection_parameter(
                    p(_h.vertex_from()), p(_h.vertex_to()),
                    p(_v_from), p(_v_to));

        return SnakeVertex { _h, lambda };
    }

    return std::optional<SnakeVertex>();
}

/**
  * Intersect one-ring boundary around _v_from with
  * straight line segment.
  */
SnakeVertex find_first_intersection(
        const Eigen::MatrixXd& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to)
{
    for (auto h_outg : _v_from.outgoing_halfedges())
    {
        const auto v = intersection_vertex(h_outg.next().opposite(), _param, _v_from, _v_to);
        if (v)
            return v.value();
    }

    LE_ERROR_THROW("Could not find first intersection.");
}

/**
 * Intersect edges of given face with line segment.
 * Ignore one edge. Return outer halfedge.
 */
SnakeVertex find_next_intersection(
        const pm::face_handle _f, // intersect edges of this face with line
        const pm::halfedge_handle _h_ignore, // ignore this edge
        const Eigen::MatrixXd& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to)
{
    for (auto h : _f.halfedges())
    {
        // Ignore edge
        if (h.edge() == _h_ignore.edge())
            continue;

        const auto v = intersection_vertex(h.opposite(), _param, _v_from, _v_to);
        if (v)
            return v.value();
    }

    LE_ERROR_THROW("Could not find next intersection.");
}

bool incident(
        const pm::face_handle& _f,
        const pm::vertex_handle& _v)
{
    for (auto v : _f.vertices())
    {
        if (v == _v)
            return true;
    }

    return false;
}

}

Snake snake_from_parametrization(
        const Eigen::MatrixXd& _param,
        const pm::vertex_handle& _v_from,
        const pm::vertex_handle& _v_to)
{
    exactinit();
    LE_ASSERT(_v_from != _v_to);

    SnakeVertex sv_from { _v_from.any_outgoing_halfedge(), 0.0 };
    SnakeVertex sv_to { _v_to.any_outgoing_halfedge(), 0.0 };

    // Vertices adjacent?
    if (pm::halfedge_from_to(_v_from, _v_to).is_valid())
        return Snake { { sv_from, sv_to } };

    Snake snake;
    snake.vertices.push_back(sv_from);

    // Find first intersected edge.
    snake.vertices.push_back(find_first_intersection(_param, _v_from, _v_to));

    // Stop if current face is incident to target vertex.
    int safeguard = 0;
    while (!incident(snake.vertices.back().h.face(), _v_to))
    {
        // Intersect current face with line segment.
        // Ignore the edge across which we entered.
        const auto h = snake.vertices.back().h;
        snake.vertices.push_back(find_next_intersection(h.face(), h, _param, _v_from, _v_to));

        LE_ASSERT(safeguard < 1e6);
        ++safeguard;
    }

    snake.vertices.push_back(sv_to);
    return snake;
}

}
