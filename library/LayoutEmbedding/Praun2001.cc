#include "Praun2001.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/IGLMesh.hh>
#include <LayoutEmbedding/UnionFind.hh>
#include <LayoutEmbedding/VirtualPort.hh>

#include <igl/harmonic.h>
#include <Eigen/Dense>
#include <set>
#include <queue>

namespace LayoutEmbedding {

/// Heuristic detection of paths that might introduce swirls after insertion.
/// For each vertex around the face that is incident to _l_he on the left,
/// a shortest path towards the given path is traced.
/// If the path is hit from the right side (instead of the left), this is considered a potential swirl.
/// Returns true if a potential swirl is detected, false otherwise.
bool swirl_detection(Embedding& _em, const pm::halfedge_handle& _l_he, const VirtualPath& _path)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::Mesh& t_m = _em.target_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    // Walk along the VertexEdgePath and mark the vertices directly left and right of it with a special attribute:
    // The indicator attribute assigns each vertex a value in {-1, 0, 1},
    // -1 meaning it is directly on the left of the arc,
    // 1 meaning it is directly on the right of the arc,
    // 0 otherwise.
    pm::vertex_attribute<int> t_indicator(t_m);

    LE_ASSERT(is_real_vertex(_path.front()));
    LE_ASSERT(is_real_vertex(_path.back()));

    for (int i = 0; i < _path.size(); ++i) {
        const auto& vv = _path[i];

        if (is_real_vertex(vv)) {
            if (i > 0 && i < _path.size() - 1) {
                const auto& el_prev = _path[i - 1];
                const auto& el_next = _path[i + 1];
                const auto& v = real_vertex(vv);

                VirtualPort vh_start{v, el_prev};
                VirtualPort vh_end{v, el_next};

                auto vh_current = vh_start.rotated_cw();
                while (vh_current != vh_end) {
                    if (is_real_vertex(vh_current.to)) {
                        const auto& v_to = real_vertex(vh_current.to);
                        t_indicator[v_to] = -1; // "Left"
                    }
                    vh_current = vh_current.rotated_cw();
                }

                while (vh_current != vh_start) {
                    if (is_real_vertex(vh_current.to)) {
                        const auto& v_to = real_vertex(vh_current.to);
                        t_indicator[v_to] = 1; // "Right"
                    }
                    vh_current = vh_current.rotated_cw();
                }
            }
        }
        else if (is_real_edge(vv)) {
            LE_ASSERT(i > 0);
            LE_ASSERT(i < _path.size() - 1);

            const auto& e = real_edge(vv);
            auto he = pm::halfedge_handle::invalid;

            const auto& vv_next = _path[i + 1];
            if (is_real_vertex(vv_next)) {
                const auto& v_next = real_vertex(vv_next);
                if (e.halfedgeA().next().vertex_to() == v_next) {
                    he = e.halfedgeA();
                }
                else if (e.halfedgeB().next().vertex_to() == v_next) {
                    he = e.halfedgeB();
                }
            }
            else if (is_real_edge(vv_next)) {
                const auto& e_next = real_edge(vv_next);

                if ((e.halfedgeA().face() == e_next.halfedgeA().face()) || (e.halfedgeA().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeA();
                }
                else if ((e.halfedgeB().face() == e_next.halfedgeA().face()) || (e.halfedgeB().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeB();
                }
            }

            LE_ASSERT(he.is_valid());
            t_indicator[he.vertex_from()] = -1; // "Left"
            t_indicator[he.vertex_to()] = 1; // "Right"
        }
    }

    // Start a shortest-path search from the seed vertices and see whether it first meets a vertex marked "Left" (good) or "Right" (bad)

    struct Candidate
    {
        double distance;
        pm::vertex_handle v;

        bool operator<(const Candidate& _rhs) const
        {
            return distance > _rhs.distance;
        }
    };

    pm::vertex_attribute<double> distance(t_m, std::numeric_limits<double>::infinity());
    std::priority_queue<Candidate> q;
    std::vector<pm::vertex_handle> t_seed_vertices;
    const auto& l_f = _l_he.face();
    for (const auto l_v : l_f.vertices()) {
        if ((l_v == _l_he.vertex_from()) || (l_v == _l_he.vertex_to())) {
            continue;
        }
        const auto t_v = _em.matching_target_vertex(l_v);
        t_seed_vertices.push_back(t_v);
        distance[t_v] = 0.0;
        q.push({0.0, t_v});
    }

    while (!q.empty()) {
        const auto c = q.top();
        q.pop();

        if (t_indicator[c.v] == -1) {
            // We arrived on the correct (left) side of the path. Probably no spiral.
            return false;
        }
        else if (t_indicator[c.v] == 1) {
            // We arrived on the wrong (right) side of the path. Spiral detected.
            return true;
        }

        for (const auto he : c.v.outgoing_halfedges()) {
            const auto& v_to = he.vertex_to();
            const double new_distance = distance[c.v] + tg::distance(t_pos[c.v], t_pos[v_to]);
            if (new_distance < distance[v_to]) {
                distance[v_to] = new_distance;
                q.push({new_distance, v_to});
            }
        }
    }
    // This will likely be never reached
    return false;
}

bool swirl_detection_bidirectional(Embedding& _em, const pm::halfedge_handle& _l_he, const VirtualPath& _path)
{
    if (swirl_detection(_em, _l_he, _path)) {
        return true;
    }
    else {
        const auto l_he_opp = _l_he.opposite();
        auto path_opp = _path;
        std::reverse(path_opp.begin(), path_opp.end());
        return swirl_detection(_em, l_he_opp, path_opp);
    }
}

Praun2001Result praun2001(Embedding& _em, const Praun2001Settings& _settings)
{
    Praun2001Result result;

    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::Mesh& t_m = _em.target_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    const int l_num_v = l_m.vertices().count();

    IGLMesh t_igl = to_igl_mesh(t_pos);
    Eigen::VectorXi b(l_num_v); // Boundary indices into t_igl.V
    {
        int b_row = 0;
        for (const auto& l_v : l_m.vertices()) {
            b[b_row] = _em.matching_target_vertex(l_v).idx.value;
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

        VirtualPath best_path;
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

            VirtualPath path = _em.find_shortest_path(l_e.halfedgeA());
            double path_cost = _em.path_length(path);

            // If we use an arbitrary insertion order, we can early-out after the first path is found
            if (_settings.insertion_order == Praun2001Settings::InsertionOrder::Arbitrary) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
                break;
            }

            if (_settings.use_swirl_detection) {
                // Only do the swirl test if the current path is already a contender.
                if (path_cost < best_path_cost) {
                    if (swirl_detection_bidirectional(_em, l_e.halfedgeA(), path)) {
                        path_cost *= penalty_factor;
                    }
                }
            }

            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
            }
        }

        std::cout << "Best path cost: " << best_path_cost << std::endl;

        result.insertion_sequence.push_back(best_l_e);
        _em.embed_path(best_l_e.halfedgeA(), best_path);
        l_v_components.merge(best_l_e.vertexA().idx.value, best_l_e.vertexB().idx.value);
        l_is_embedded[best_l_e] = true;
        ++l_num_embedded_edges;
    }
    return result;
}

}
