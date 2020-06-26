#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

#include <Eigen/Dense>

struct IGLMesh
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

IGLMesh to_igl_mesh(const pm::vertex_attribute<tg::pos3>& _pos);
