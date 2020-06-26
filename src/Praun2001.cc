#include "Praun2001.hh"

#include "IGLMesh.hh"
#include "UnionFind.hh"

#include <igl/harmonic.h>
#include <Eigen/Dense>
#include <set>
#include <queue>

/// Heuristic detection of paths that might introduce swirls after insertion.
/// For each vertex around the face that is incident to _l_he on the left,
/// a shortest path towards the given path is traced.
/// If the path is hit from the right side (instead of the left), this is considered a potential swirl.
/// Returns true if a potential swirl is detected, false otherwise.
bool swirl_detection(Embedding& _em, const pm::halfedge_handle& _l_he, const VertexEdgePath& _path)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    // Walk along the VertexEdgePath and mark the vertices directly left and right of it with a special attribute:
    // The indicator attribute assigns each vertex a value in {-1, 0, 1},
    // -1 meaning it is directly on the left of the arc,
    // 1 meaning it is directly on the right of the arc,
    // 0 otherwise.
    pm::vertex_attribute<int> t_indicator(t_m);

    assert(std::holds_alternative<pm::vertex_handle>(_path.front()));
    assert(std::holds_alternative<pm::vertex_handle>(_path.back()));

    struct VirtualHalfedge
    {
        pm::vertex_handle from;
        VertexEdgeElement to;

        bool operator==(const VirtualHalfedge& _rhs) const
        {
            return (from == _rhs.from) && (to == _rhs.to);
        }

        bool operator!=(const VirtualHalfedge& _rhs) const
        {
            return !(*this == _rhs);
        }

        VirtualHalfedge rotated_cw() const
        {
            if (std::holds_alternative<pm::vertex_handle>(to)) {
                const auto he = pm::halfedge_from_to(from, std::get<pm::vertex_handle>(to));
                const auto e_new = he.opposite().prev().edge();
                return {from, e_new};
            }
            else if (std::holds_alternative<pm::edge_handle>(to)) {
                const auto e = std::get<pm::edge_handle>(to);
                auto he = pm::halfedge_handle::invalid;
                if (e.halfedgeA().next().vertex_to() == from) {
                    he = e.halfedgeA();
                }
                else if (e.halfedgeB().next().vertex_to() == from) {
                    he = e.halfedgeB();
                }
                assert(he.is_valid());
                const auto v_new = he.vertex_from();
                return {from, v_new};
            }
            assert(false);
            return {};
        }
    };

    for (int i = 0; i < _path.size(); ++i) {
        const auto& el = _path[i];

        if (std::holds_alternative<pm::vertex_handle>(el)) {
            if (i > 0 && i < _path.size() - 1) {
                const auto& el_prev = _path[i - 1];
                const auto& el_next = _path[i + 1];
                const auto& v = std::get<pm::vertex_handle>(el);

                VirtualHalfedge vh_start{v, el_prev};
                VirtualHalfedge vh_end{v, el_next};

                auto vh_current = vh_start.rotated_cw();
                while (vh_current != vh_end) {
                    if (std::holds_alternative<pm::vertex_handle>(vh_current.to)) {
                        const auto& v_to = std::get<pm::vertex_handle>(vh_current.to);
                        t_indicator[v_to] = -1; // "Left"
                    }
                    vh_current = vh_current.rotated_cw();
                }

                while (vh_current != vh_start) {
                    if (std::holds_alternative<pm::vertex_handle>(vh_current.to)) {
                        const auto& v_to = std::get<pm::vertex_handle>(vh_current.to);
                        t_indicator[v_to] = 1; // "Right"
                    }
                    vh_current = vh_current.rotated_cw();
                }
            }
        }
        else if (std::holds_alternative<pm::edge_handle>(el)) {
            assert(i > 0);
            assert(i < _path.size() - 1);

            const auto& e = std::get<pm::edge_handle>(el);
            auto he = pm::halfedge_handle::invalid;

            const auto& el_next = _path[i + 1];
            if (std::holds_alternative<pm::vertex_handle>(el_next)) {
                const auto& v_next = std::get<pm::vertex_handle>(el_next);
                if (e.halfedgeA().next().vertex_to() == v_next) {
                    he = e.halfedgeA();
                }
                else if (e.halfedgeB().next().vertex_to() == v_next) {
                    he = e.halfedgeB();
                }
            }
            else if (std::holds_alternative<pm::edge_handle>(el_next)) {
                const auto& e_next = std::get<pm::edge_handle>(el_next);

                if ((e.halfedgeA().face() == e_next.halfedgeA().face()) || (e.halfedgeA().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeA();
                }
                else if ((e.halfedgeB().face() == e_next.halfedgeA().face()) || (e.halfedgeB().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeB();
                }
            }

            assert(he.is_valid());
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
        const auto t_v = _em.l_matching_vertex[l_v];
        t_seed_vertices.push_back(t_v);
        distance[t_v] = 0.0;
        q.push({0.0, t_v});
    }
    std::cout << "Starting swirl test from " << t_seed_vertices.size() << " vertices." << std::endl;

    while (!q.empty()) {
        const auto c = q.top();
        q.pop();

        if (t_indicator[c.v] == -1) {
            // We arrived on the correct (left) side of the path. Probably no spiral.
            std::cout << "No Swirl" << std::endl;
            return false;
        }
        else if (t_indicator[c.v] == 1) {
            // We arrived on the wrong (right) side of the path. Spiral detected.
            std::cout << "Swirl!!!" << std::endl;
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

void praun2001(Embedding& _em)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    const int l_num_v = l_m.vertices().count();

    IGLMesh t_igl = to_igl_mesh(t_pos);
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

            // Only do the swirl test if the current path is already a contender.
            if (path_cost < best_path_cost) {
                if (swirl_detection(_em, l_e.halfedgeA(), path)) {
                    path_cost *= penalty_factor;
                }
            }

            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
            }
        }

        std::cout << "Best path cost: " << best_path_cost << std::endl;

        embed_path(_em, best_l_e.halfedgeA(), best_path);
        l_v_components.merge(best_l_e.vertexA().idx.value, best_l_e.vertexB().idx.value);
        l_is_embedded[best_l_e] = true;
        ++l_num_embedded_edges;
    }
}
