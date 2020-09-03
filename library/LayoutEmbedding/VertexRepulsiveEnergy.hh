#pragma once

#include <LayoutEmbedding/Embedding.hh>

#include <Eigen/Dense>

namespace LayoutEmbedding {

Eigen::MatrixXd compute_vertex_repulsive_energy(const Embedding& _em);

}
