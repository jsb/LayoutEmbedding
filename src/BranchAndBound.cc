#include "BranchAndBound.hh"

#include <Assert.hh>
#include <UnionFind.hh>

#include <iostream> // DEBUG
#include <queue>

struct Candidate
{
    double lower_bound;
    double priority;
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

    // Measure length of "embedded" edges
    std::set<pm::edge_index> embedded_l_e;
    double embedded_cost = 0.0;
    for (const auto& l_e : _insertions) {
        auto l_he = l_e.halfedgeA();
        auto path = find_shortest_path(em, l_he);

        embedded_cost += path_length(em, path);

        embed_path(em, l_he, path);
        embedded_l_e.insert(l_e);
    }

    std::set<pm::edge_index> conflicting_l_e;
    VertexEdgeAttribute<std::set<pm::edge_index>> covered(*t_m_copy);

    // Measure length of "unembedded" edges
    double unembedded_cost = 0.0;
    for (const auto& l_e : l_m.edges()) {
        if (embedded_l_e.count(l_e)) {
            continue;
        }

        const auto& path = find_shortest_path(em, l_e);
        if (path.empty()) {
            unembedded_cost = std::numeric_limits<double>::infinity();
        }
        else {
            unembedded_cost += path_length(em, path);
        }

        // Mark conflicting edges
        for (int i = 1; i < path.size() - 1; ++i) {
            const auto& el = path[i];
            for (const auto& l_e_other : covered[el]) {
                conflicting_l_e.insert(l_e);
                conflicting_l_e.insert(l_e_other);
            }
            covered[el].insert(l_e);
        }
    }

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

    const double max_gap = 0.10;

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
        if (gap <= max_gap) {
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
        UnionFind l_v_components(l_m.vertices().size());

        double embedded_cost = 0.0;
        for (const auto& l_e : c.insertions) {
            auto l_he = l_e.halfedgeA();
            auto path = find_shortest_path(em, l_he);

            embedded_cost += path_length(em, path);

            embed_path(em, l_he, path);
            embedded_l_e.insert(l_e);

            int l_v_a_i = l_e.vertexA().idx.value;
            int l_v_b_i = l_e.vertexB().idx.value;
            LE_ASSERT(!l_v_components.equivalent(l_v_a_i, l_v_b_i));
            l_v_components.merge(l_v_a_i, l_v_b_i);
        }

        // Classify the candidate paths: conflicting, blocked, nonconflicting
        VertexEdgeAttribute<std::set<pm::edge_index>> covered(*t_m_copy);

        std::set<pm::edge_index> conflicting_l_e;
        std::set<pm::edge_index> blocked_l_e;

        double unembedded_cost = 0.0;
        for (const auto& l_e : l_m.edges()) {
            if (embedded_l_e.count(l_e)) {
                continue;
            }

            const auto& path = find_shortest_path(em, l_e);
            unembedded_cost += path_length(em, path);

            const int l_v_a_id = l_e.vertexA().idx.value;
            const int l_v_b_id = l_e.vertexB().idx.value;
            if (l_v_components.equivalent(l_v_a_id, l_v_b_id)) {
                blocked_l_e.insert(l_e);
                continue;
            }

            for (int i = 1; i < path.size() - 1; ++i) {
                const auto& el = path[i];
                for (const auto& l_e_other : covered[el]) {
                    conflicting_l_e.insert(l_e);
                    conflicting_l_e.insert(l_e_other);
                }
                covered[el].insert(l_e);
            }
        }

        std::set<pm::edge_index> non_conflicting_l_e;
        for (const auto& l_e : l_m.edges()) {
            if (!embedded_l_e.count(l_e) &&!blocked_l_e.count(l_e) && !conflicting_l_e.count(l_e)) {
                non_conflicting_l_e.insert(l_e);
            }
        }

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

                    //new_c.priority = new_c.lower_bound / new_c.insertions.size();
                    new_c.priority = new_c.lower_bound * stats.num_conflicting;

                    const double new_gap = 1.0 - new_c.lower_bound / global_upper_bound;
                    if (new_gap > max_gap) {
                        q.push(new_c);
                    }
                }
            }
        }
    }

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
