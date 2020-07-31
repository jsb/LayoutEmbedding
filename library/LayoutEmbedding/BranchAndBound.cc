#include "BranchAndBound.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>
#include <LayoutEmbedding/CyclicOrderSentinel.hh>

// DEBUG
#include <glow-extras/viewer/view.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>
#include <iostream>

#include <queue>

namespace LayoutEmbedding {

// TODO: Hide this nasty code in a separate implementation file.

struct VEIntersectionCache
{
    const pm::Mesh& m;

    using Segment = std::pair<VertexEdgeElement, VertexEdgeElement>;
    using Label = pm::edge_index;
    using LabelSet = std::set<Label>;

    pm::vertex_attribute<Label> v_label;
    pm::edge_attribute<Label> e_label;
    pm::face_attribute<Label> f_label;
    LabelSet global_conflicts;

    explicit VEIntersectionCache(const pm::Mesh& _m) :
        m(_m),
        v_label(m),
        e_label(m),
        f_label(m)
    {
    }

    void insert(const pm::vertex_handle& _v, const Label& _l)
    {
        if (v_label[_v].is_valid()) {
            global_conflicts.insert(_l);
            global_conflicts.insert(v_label[_v]);
        }
        v_label[_v] = _l;
    }

    void insert(const pm::edge_handle& _e, const Label& _l)
    {
        if (e_label[_e].is_valid()) {
            global_conflicts.insert(_l);
            global_conflicts.insert(e_label[_e]);
        }
        e_label[_e] = _l;
    }

    void insert(const pm::face_handle& _f, const Label& _l)
    {
        if (f_label[_f].is_valid()) {
            global_conflicts.insert(_l);
            global_conflicts.insert(f_label[_f]);
        }
        f_label[_f] = _l;
    }

    void insert_element(const VertexEdgeElement& _el, const Label& _l)
    {
        if (is_vertex(_el)) {
            insert(vertex(_el), _l);
        }
        else {
            insert(edge(_el), _l);
        }
    }

    void insert_segment(const VertexEdgeElement& _el0, const VertexEdgeElement& _el1, const Label& _l)
    {
        if (is_vertex(_el0)) {
            if (is_vertex(_el1)) {
                // (V,V) case
                const auto& v0 = vertex(_el0);
                const auto& v1 = vertex(_el1);
                const auto& he = pm::halfedge_from_to(v0, v1);
                LE_ASSERT(he.is_valid());
                const auto& e = he.edge();
                insert(e, _l);
            }
            else {
                // (V,E) case
                const auto& v = vertex(_el0);
                const auto& e = edge(_el1);
                const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
                LE_ASSERT(f.is_valid());
                insert(f, _l);
            }
        }
        else {
            if (is_vertex(_el1)) {
                // (E,V) case
                const auto& e = edge(_el0);
                const auto& v = vertex(_el1);
                const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
                LE_ASSERT(f.is_valid());
                insert(f, _l);
            }
            else {
                // (E,E) case
                const auto& e0 = edge(_el0);
                const auto& e1 = edge(_el1);
                const auto& f = common_face(e0, e1);
                LE_ASSERT(f.is_valid());
                insert(f, _l);
            }
        }
    }

    void insert_path(const VertexEdgePath& _path, const Label& _l)
    {
        // Path elements ("virtual vertices")
        LE_ASSERT(_path.size() >= 2);
        // Note: We deliberately skip the first and last element
        for (int i = 1; i < _path.size() - 1; ++i) {
            insert_element(_path[i], _l);
        }

        // Path segments ("virtual edges")
        for (int i = 0; i < _path.size() - 1; ++i) {
            insert_segment(_path[i], _path[i+1], _l);
        }
    }
};

std::set<pm::edge_index> conflicting_paths(const Embedding& _em, const pm::edge_attribute<VertexEdgePath>& _paths)
{
    // Rules:
    // - Paths must not intersect.
    // - Paths are allowed to touch other paths at layout vertices (path endpoints).
    // - Around each layout vertex, the cyclic order of outgoing edges must be consistent with that in the layout.

    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;

    VEIntersectionCache c(t_m);

    for (const auto& l_e : l_m.edges()) {
        const auto& path = _paths[l_e];
        if (path.empty()) {
            // Layout edges that have an already embedded path are not candidates.
            // Therefore, their candidate path is left empty.
            continue;
        }
        LE_ASSERT(path.size() >= 2);
        c.insert_path(path, l_e);
    }

    // TODO: detect additional conflicts due to cyclic order!

    return c.global_conflicts;
}

struct Candidate
{
    double lower_bound = std::numeric_limits<double>::infinity();
    double priority = 0.0;
    std::vector<pm::edge_handle> insertions;

    bool operator<(const Candidate& _rhs) const
    {
        return priority > _rhs.priority;
    }
};

struct EmbeddingStats
{
    double embedded_cost;
    double unembedded_cost;
    int num_embedded;
    int num_conflicting;
    int num_non_conflicting;
};

EmbeddingStats calc_cost_lower_bound(const Embedding& _em, const std::vector<pm::edge_handle>& _insertions)
{
    // Copy embedding
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    pm::unique_ptr<pm::Mesh> t_m_copy = t_m.copy();
    auto t_pos_copy = t_m_copy->vertices().make_attribute<tg::pos3>();
    t_pos_copy.copy_from(t_pos);

    RefinableMesh rm = make_refinable_mesh(*t_m_copy, t_pos_copy);
    Embedding em = make_embedding(l_m, rm);

    for (const auto& l_v : l_m.vertices()) {
        em.l_matching_vertex[l_v.idx] = (*t_m_copy)[_em.l_matching_vertex[l_v.idx].idx];
    }
    for (const auto& t_v : t_m_copy->vertices()) {
        em.t_matching_vertex[t_v.idx] = l_m[_em.t_matching_vertex[t_v.idx].idx];
    }
    for (const auto& t_he : t_m_copy->halfedges()) {
        em.t_matching_halfedge[t_he.idx] = l_m[_em.t_matching_halfedge[t_he.idx].idx];
    }

    // Measure length of "embedded" edges
    std::set<pm::edge_index> embedded_l_e;
    double embedded_cost = 0.0;
    for (const auto& l_e : _insertions) {
        if (is_embedded(em, l_e)) {
            embedded_l_e.insert(l_e);
            embedded_cost += embedded_path_length(em, l_e);
        }
    }

    // Measure length of "unembedded" edges
    pm::edge_attribute<VertexEdgePath> candidate_paths(l_m);
    double unembedded_cost = 0.0;
    for (const auto& l_e : l_m.edges()) {
        if (embedded_l_e.count(l_e)) {
            continue;
        }

        const auto& path = find_shortest_path(em, l_e);
        if (path.empty()) {
            unembedded_cost = std::numeric_limits<double>::infinity();
            break;
        }
        else {
            unembedded_cost += path_length(em, path);
            candidate_paths[l_e] = path;
        }
    }

    std::set<pm::edge_index> conflicting_l_e = conflicting_paths(em, candidate_paths);

    EmbeddingStats result;

    result.num_embedded = _insertions.size();
    result.num_conflicting = conflicting_l_e.size();
    result.num_non_conflicting = l_m.edges().size() - result.num_embedded - result.num_conflicting;

    result.embedded_cost = embedded_cost;
    result.unembedded_cost = unembedded_cost;

    return result;
}

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    double global_upper_bound = std::numeric_limits<double>::infinity();
    // TODO: Run heuristic algorithm to find a tighter initial upper bound

    Candidate best_solution; // initially empty

    std::priority_queue<Candidate> q;

    {
        Candidate c;
        c.lower_bound = 0.0;
        c.priority = 0.0;
        c.insertions = {};
        q.push(c);
    }

    while (!q.empty()) {
        auto c = q.top();
        q.pop();

        const double gap = 1.0 - c.lower_bound / global_upper_bound;
        if (gap <= _settings.optimality_gap) {
            continue;
        }

        // Reconstruct the embedding associated with this embedding sequence
        pm::unique_ptr<pm::Mesh> t_m_copy = t_m.copy();
        auto t_pos_copy = t_m_copy->vertices().make_attribute<tg::pos3>();
        t_pos_copy.copy_from(t_pos);

        RefinableMesh rm = make_refinable_mesh(*t_m_copy, t_pos_copy);
        Embedding em = make_embedding(l_m, rm);

        for (const auto& l_v : l_m.vertices()) {
            em.l_matching_vertex[l_v.idx] = (*t_m_copy)[_em.l_matching_vertex[l_v.idx].idx];
        }
        for (const auto& t_v : t_m_copy->vertices()) {
            em.t_matching_vertex[t_v.idx] = l_m[_em.t_matching_vertex[t_v.idx].idx];
        }

        // Embed the "already embedded" edges
        std::set<pm::edge_index> embedded_l_e;

        double embedded_cost = 0.0;
        for (const auto& l_e : c.insertions) {
            auto l_he = l_e.halfedgeA();
            auto path = find_shortest_path(em, l_he);
            if (path.empty()) {
                embedded_cost = std::numeric_limits<double>::infinity();
                break;
            }

            embedded_cost += path_length(em, path);

            embed_path(em, l_he, path);
            embedded_l_e.insert(l_e);
        }
        if (std::isinf(embedded_cost)) {
            continue;
        }

        // Classify the candidate paths: conflicting and nonconflicting
        pm::edge_attribute<VertexEdgePath> candidate_paths(l_m);

        double unembedded_cost = 0.0;
        for (const auto& l_e : l_m.edges()) {
            if (embedded_l_e.count(l_e)) {
                continue;
            }

            const auto& path = find_shortest_path(em, l_e);
            if (path.empty()) {
                unembedded_cost = std::numeric_limits<double>::infinity();
                break;
            }
            unembedded_cost += path_length(em, path);

            LE_ASSERT(path.size() >= 2);
            candidate_paths[l_e] = path;
        }
        if (std::isinf(unembedded_cost)) {
            continue;
        }

        std::set<pm::edge_index> conflicting_l_e = conflicting_paths(em, candidate_paths);

        // Additional conflicts may arise from inconsistent ordering of outgoing edges around vertices.
        /*
        for (const auto& l_he : l_m.halfedges()) {
            const auto& l_e = l_he.edge();
            if (embedded_l_e.count(l_e) || conflicting_l_e.count(l_e)) {
                continue;
            }

            // Layout edges at the same vertex coming before and after the current edge l_e (in CCW order)
            const auto& l_he_before = l_he.opposite().next();
            const auto& l_he_after = l_he.prev().opposite();
            const auto& l_e_before = l_he_before.edge();
            const auto& l_e_after = l_he_after.edge();

            CyclicOrderSentinel<pm::edge_index> sentinel({l_e_before, l_e, l_e_after});

            // TODO: use values from covered[el] to find out the embeddings of the surrounding edges.
            const auto& l_v = l_he.vertex_from();
            const auto& t_v = em.l_matching_vertex[l_v];
            for (const auto& t_he : t_v.outgoing_halfedges()) {
                const auto& covering_l_es = covered[t_he.edge()];
                for (const auto& c_l_e : covering_l_es) {
                    sentinel.encounter(c_l_e);
                }
                if (!sentinel.valid()) {
                    break;
                }
            }
            if (!sentinel.valid()) {
                conflicting_l_e.insert(l_e);
                break;
            }
        }
        */

        std::set<pm::edge_index> non_conflicting_l_e;
        for (const auto& l_e : l_m.edges()) {
            if (!embedded_l_e.count(l_e) && !conflicting_l_e.count(l_e)) {
                non_conflicting_l_e.insert(l_e);
            }
        }

        LE_ASSERT(embedded_l_e.size() + conflicting_l_e.size() + non_conflicting_l_e.size() == l_m.edges().size());

        const double cost_lower_bound = embedded_cost + unembedded_cost;

        std::cout << "|Embd|: " << c.insertions.size();
        std::cout << "    ";
        std::cout << "|Conf|: " << conflicting_l_e.size();
        std::cout << "    ";
        std::cout << "|Ncnf|: " << non_conflicting_l_e.size();
        std::cout << "    ";
        std::cout << "LB: " << cost_lower_bound;
        std::cout << "    ";
        std::cout << "UB: " << global_upper_bound;
        std::cout << "    ";
        std::cout << "gap: " << (gap * 100.0) << " %";
        std::cout << "    ";
        std::cout << "|Q|: " << q.size();
        std::cout << std::endl;

        // Completed layout?
        if (conflicting_l_e.empty()) {
            if (cost_lower_bound < global_upper_bound) {
                global_upper_bound = cost_lower_bound;
                best_solution = c;
                std::cout << "New upper bound: " << global_upper_bound << std::endl;
            }
        }
        else {
            // Add children to the queue
            if (cost_lower_bound < global_upper_bound) {
                for (const auto& l_e : conflicting_l_e) {
                    Candidate new_c = c;
                    new_c.insertions.push_back(l_m.edges()[l_e]);

                    EmbeddingStats stats = calc_cost_lower_bound(em, new_c.insertions);
                    new_c.lower_bound = stats.embedded_cost + stats.unembedded_cost;

                    new_c.priority = new_c.lower_bound * stats.num_conflicting;

                    const double new_gap = 1.0 - new_c.lower_bound / global_upper_bound;
                    if (new_gap > _settings.optimality_gap) {
                        q.push(new_c);
                    }
                }
            }
        }
    }

    std::cout << "Branch-and-bound optimization completed." << std::endl;

    // Apply the victorious embedding sequence to the input embedding

    // Edges with predefined insertion sequence
    std::set<pm::edge_index> l_e_embedded;
    for (const auto& l_e : best_solution.insertions) {
        const auto l_he = l_e.halfedgeA();
        const auto path = find_shortest_path(_em, l_he);
        embed_path(_em, l_he, path);
        l_e_embedded.insert(l_e);
    }
    // Remaining edges
    for (const auto& l_e : l_m.edges()) {
        if (!l_e_embedded.count(l_e)) {
            const auto l_he = l_e.halfedgeA();
            const auto path = find_shortest_path(_em, l_he);
            embed_path(_em, l_he, path);
            l_e_embedded.insert(l_e);
        }
    }
}

}
