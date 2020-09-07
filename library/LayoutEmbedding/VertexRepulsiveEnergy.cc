#include "VertexRepulsiveEnergy.hh"

#include <LayoutEmbedding/Harmonic.hh>

namespace LayoutEmbedding {

Eigen::MatrixXd compute_vertex_repulsive_energy(const Embedding& _em)
{
    const int l_num_v = _em.layout_mesh().vertices().size();
    const int t_num_v = _em.target_mesh().vertices().size();

    // Set up boundary conditions
    auto constrained = _em.target_mesh().vertices().make_attribute<bool>(false);
    Eigen::MatrixXd constraint_values = Eigen::MatrixXd::Zero(t_num_v, l_num_v);

    for (const auto l_v : _em.layout_mesh().vertices())
    {
        const auto t_v = _em.matching_target_vertex(l_v);
        constrained[t_v] = true;
        constraint_values(t_v.idx.value, l_v.idx.value) = 1.0;
    }

    // Compute l_num_v many harmonic fields
    Eigen::MatrixXd W;
    LE_ASSERT(harmonic(_em.target_pos(), constrained, constraint_values, W));

    return W;
}

}
