#include "BranchAndBound.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>
#include <LayoutEmbedding/VirtualPathConflictSentinel.hh>

#include <chrono>
#include <queue>

namespace LayoutEmbedding {

std::set<pm::edge_index> conflicting_paths(const Embedding& _em, const pm::edge_attribute<VirtualPath>& _paths)
{
    // Rules:
    // - Paths must not intersect.
    // - Paths are allowed to touch other paths at layout vertices (path endpoints).
    // - Around each layout vertex, the cyclic order of outgoing edges must be consistent with that in the layout.

    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m;

    VirtualPathConflictSentinel vpcs(t_m);

    for (const auto& l_e : l_m.edges()) {
        const auto& path = _paths[l_e];
        if (path.empty()) {
            // Layout edges that have an already embedded path are not candidates.
            // Therefore, their candidate path is left empty.
            continue;
        }
        LE_ASSERT(path.size() >= 2);
        vpcs.insert_path(path, l_e);
    }

    // TODO: detect additional conflicts due to cyclic order!

    return vpcs.global_conflicts;
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
    const pm::Mesh& t_m = *_em.t_m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_pos;

    pm::unique_ptr<pm::Mesh> t_m_copy = t_m.copy();
    auto t_pos_copy = t_m_copy->vertices().make_attribute<tg::pos3>();
    t_pos_copy.copy_from(t_pos);

    Embedding em = make_embedding(l_m, *t_m_copy, t_pos_copy);

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
    pm::edge_attribute<VirtualPath> candidate_paths(l_m);
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
    const pm::Mesh& t_m = *_em.t_m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_pos;

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

    const auto start_time = std::chrono::steady_clock::now();

    while (!q.empty()) {
        // Timeout
        if (_settings.timeout > 0.0) {
            const auto current_time = std::chrono::steady_clock::now();
            const auto interval = current_time - start_time;
            using seconds_double = std::chrono::duration<double>;
            const double elapsed_seconds = std::chrono::duration_cast<seconds_double>(interval).count();
            if (elapsed_seconds >= _settings.timeout) {
                std::cout << "Reached time limit of " << _settings.timeout << " s. Terminating." << std::endl;
                if (std::isinf(global_upper_bound)) {
                    std::cout << "Warning: No valid solution was found during that time." << std::endl;
                }
                break;
            }
        }

        auto c = q.top();
        q.pop();

        double gap = 1.0 - c.lower_bound / global_upper_bound;
        if (gap <= _settings.optimality_gap) {
            continue;
        }

        // Reconstruct the embedding associated with this embedding sequence
        pm::unique_ptr<pm::Mesh> t_m_copy = t_m.copy();
        auto t_pos_copy = t_m_copy->vertices().make_attribute<tg::pos3>();
        t_pos_copy.copy_from(t_pos);

        Embedding em = make_embedding(l_m, *t_m_copy, t_pos_copy);

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
        pm::edge_attribute<VirtualPath> candidate_paths(l_m);

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

        // TODO: Shouldn't these values always match? Bug?
        //LE_ASSERT(cost_lower_bound == c.lower_bound);
        gap = 1.0 - cost_lower_bound / global_upper_bound;
        if (gap <= _settings.optimality_gap) {
            continue;
        }

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
