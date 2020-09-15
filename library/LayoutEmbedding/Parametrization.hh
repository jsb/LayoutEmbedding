#pragma once

#include <Eigen/Dense>
#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

namespace LayoutEmbedding
{

using VertexParam = pm::vertex_attribute<tg::dpos2>;
using HalfedgeParam = pm::halfedge_attribute<tg::dpos2>;

bool injective(
        const VertexParam& _param);

}
