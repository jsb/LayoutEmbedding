#include "ApproximateGeodesicDistance.hh"

#include <LayoutEmbedding/IGLMesh.hh>
#include <LayoutEmbedding/Util/Assert.hh>

#include <igl/heat_geodesics.h>

namespace LayoutEmbedding {

pm::vertex_attribute<double> approximate_geodesic_distance(const pm::vertex_attribute<tg::pos3>& _pos, const std::vector<pm::vertex_handle>& _source_vertices)
{
    IGLMesh im = to_igl_mesh(_pos);

    igl::HeatGeodesicsData<double> data;
    igl::heat_geodesics_precompute(im.V, im.F, data);

    // Build vector of source vertex indices
    Eigen::VectorXi gamma(_source_vertices.size());
    for (int row = 0; row < gamma.size(); ++row) {
        const auto& v = _source_vertices[row];
        LE_ASSERT(v.mesh == &_pos.mesh());
        gamma[row] = v.idx.value;
    }

    Eigen::VectorXd D;
    igl::heat_geodesics_solve(data, gamma, D);

    const auto& m = _pos.mesh();
    auto result = m.vertices().make_attribute<double>();
    for (const auto& v : m.vertices()) {
        result[v] = D[v.idx.value];
    }
    return result;
}

pm::vertex_attribute<double> approximate_geodesic_distance(const pm::vertex_attribute<tg::pos3>& _pos, const pm::vertex_handle& _source_vertex)
{
    return approximate_geodesic_distance(_pos, std::vector<pm::vertex_handle>{_source_vertex});
}

}
