#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

#include <Eigen/Dense>

struct igl_mesh
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

igl_mesh to_igl_mesh(const pm::vertex_attribute<tg::pos3>& _pos);
