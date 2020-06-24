#include "praun2001.hh"

#include "igl_mesh.hh"
#include "UnionFind.hh"

#include <igl/harmonic.h>
#include <Eigen/Dense>
#include <set>

void praun2001(Embedding& _em)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    const int l_num_v = l_m.vertices().count();

    igl_mesh t_igl = to_igl_mesh(t_pos);
    Eigen::VectorXi b(l_num_v); // Boundary indices into t_igl.V
    {
        int b_row = 0;
        for (const auto& l_v : l_m.vertices()) {
            b[b_row] = _em.l_matching_vertex[l_v].idx.value;
            ++b_row;
        }
    }
    Eigen::MatrixXd bc(l_num_v, l_num_v);
    bc.setIdentity();

    Eigen::MatrixXd W; // Output
    igl::harmonic(t_igl.V, t_igl.F, b, bc, 1, W);

    // Normalize rows of W
    for (int i = 0; i < W.rows(); ++i) {
        W.row(i) /= W.row(i).sum();
    }

    // If edges fail the "Swirl Test", their score will receive a penalty so they are processed later.
    const double penalty_factor = 2.0; // This is a guess. The exact value is never mentioned in [Praun2001].
    pm::edge_attribute<bool> l_penalty(l_m);

    pm::edge_attribute<bool> l_is_embedded(l_m);
    const int l_num_vertices = l_m.vertices().size();
    const int l_num_edges = l_m.edges().size();
    int l_num_embedded_edges = 0;

    UnionFind l_v_components(l_m.vertices().count());

    while (l_num_embedded_edges < l_num_edges) {
        std::cout << "Embedding edge " << (l_num_embedded_edges + 1) << " / " << l_num_edges << std::endl;

        VertexEdgePath best_path;
        double best_path_cost = std::numeric_limits<double>::infinity();
        pm::edge_handle best_l_e = pm::edge_handle::invalid;

        const bool is_spanning_tree = (l_num_embedded_edges >= l_num_vertices - 1);

        for (const auto& l_e : l_m.edges()) {
            if (l_is_embedded[l_e]) {
                continue;
            }

            int l_vi_a = l_e.vertexA().idx.value;
            int l_vi_b = l_e.vertexB().idx.value;

            if (!is_spanning_tree) {
                if (l_v_components.equivalent(l_vi_a, l_vi_b)) {
                    continue;
                }
            }

            VertexEdgePath path = find_shortest_path(_em, l_e.halfedgeA());
            double path_cost = path_length(_em, path);

            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
            }
        }

        std::cout << "Best path cost: " << best_path_cost << std::endl;

        insert_path(_em, best_l_e.halfedgeA(), best_path);
        l_v_components.merge(best_l_e.vertexA().idx.value, best_l_e.vertexB().idx.value);
        l_is_embedded[best_l_e] = true;
        ++l_num_embedded_edges;
    }
}
