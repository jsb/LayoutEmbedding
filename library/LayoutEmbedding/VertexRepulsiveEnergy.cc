#include "VertexRepulsiveEnergy.hh"

#include <LayoutEmbedding/IGLMesh.hh>

#include <igl/intrinsic_delaunay_cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/harmonic.h>

namespace LayoutEmbedding {

Eigen::MatrixXd compute_vertex_repulsive_energy(const Embedding& _em)
{
    const int l_num_v = _em.layout_mesh().vertices().size();
    IGLMesh t_igl = to_igl_mesh(_em.target_pos());

    Eigen::VectorXi b(l_num_v); // Boundary indices into t_igl.V
    {
        int b_row = 0;
        for (const auto l_v : _em.layout_mesh().vertices()) {
            b[b_row] = _em.matching_target_vertex(l_v).idx.value;
            ++b_row;
        }
    }
    Eigen::MatrixXd bc(l_num_v, l_num_v);
    bc.setIdentity();

    Eigen::SparseMatrix<double> L;
    igl::intrinsic_delaunay_cotmatrix(t_igl.V, t_igl.F, L);

    Eigen::SparseMatrix<double> M;
    igl::massmatrix(t_igl.V, t_igl.F, igl::MASSMATRIX_TYPE_VORONOI, M);

    Eigen::MatrixXd W;
    igl::harmonic(L, M, b, bc, 1, W);
    return W;
}

}
