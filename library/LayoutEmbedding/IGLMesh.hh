#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

#include <Eigen/Dense>

namespace LayoutEmbedding {

struct IGLMesh
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

IGLMesh to_igl_mesh(const pm::vertex_attribute<tg::pos3>& _pos);

void from_igl_mesh(const IGLMesh& _igl, pm::Mesh& _m, pm::vertex_attribute<tg::dpos3>& _pos);
void from_igl_mesh(const IGLMesh& _igl, pm::Mesh& _m, pm::vertex_attribute<tg::dpos2>& _pos);

}
