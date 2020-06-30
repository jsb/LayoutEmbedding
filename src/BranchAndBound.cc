#include "BranchAndBound.hh"

#include <UnionFind.hh>

#include <iostream> // DEBUG
#include <queue>

struct Candidate
{
    double lower_bound;
    std::vector<pm::edge_handle> insertions;

    bool operator<(const Candidate& _rhs) const
    {
        return lower_bound > _rhs.lower_bound;
    }
};

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
        c.insertions = {};
        q.push(c);
    }

    while (!q.empty()) {
        std::cout << q.size() << " items in q" << std::endl;
        auto c = q.top();
        q.pop();

        if (c.lower_bound >= global_upper_bound) {
            // Done. All following candidates will only have higher lower bounds.
            break;
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
            assert(!l_v_components.equivalent(l_v_a_i, l_v_b_i));
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

            const int l_v_a_id = l_e.vertexA().idx.value;
            const int l_v_b_id = l_e.vertexB().idx.value;
            if (l_v_components.equivalent(l_v_a_id, l_v_b_id)) {
                blocked_l_e.insert(l_e);
                continue;
            }

            const auto& path = find_shortest_path(em, l_e);
            for (int i = 1; i < path.size() - 1; ++i) {
                const auto& el = path[i];
                for (const auto& l_e_other : covered[el]) {
                    conflicting_l_e.insert(l_e);
                    conflicting_l_e.insert(l_e_other);
                }
                covered[el].insert(l_e);
            }
            unembedded_cost += path_length(em, path);
        }

        std::set<pm::edge_index> non_conflicting_l_e;
        for (const auto& l_e : l_m.edges()) {
            if (!embedded_l_e.count(l_e) &&!blocked_l_e.count(l_e) && !conflicting_l_e.count(l_e)) {
                non_conflicting_l_e.insert(l_e);
            }
        }

        const double cost_lower_bound = embedded_cost + unembedded_cost;

        // Completed layout?
        if (conflicting_l_e.empty()) {
            std::cout << "Completed layout" << std::endl;

            if (cost_lower_bound < global_upper_bound) {
                global_upper_bound = cost_lower_bound;
                std::cout << "New upper bound: " << global_upper_bound << std::endl;
                best_solution = c;
            }
        }
        else {
            std::cout << "LB: " << cost_lower_bound << ",   UB: " << global_upper_bound << std::endl;
            // Add children to the queue
            if (cost_lower_bound < global_upper_bound) {
                for (const auto& l_e : conflicting_l_e) {
                    Candidate new_c = c;
                    new_c.insertions.push_back(l_m.edges()[l_e]);
                    new_c.lower_bound = cost_lower_bound; // TODO: this could be made tighter
                    q.push(new_c);
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

void branch_and_bound2(Embedding& _em, const BranchAndBoundSettings& _settings)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;

    std::set<pm::edge_index> embedded_l_e;

    UnionFind l_v_components(l_m.vertices().size());

    int num_decisions = 0;
    double num_potential_leaves = 1;

    std::srand(time(nullptr));

    while (true) {
        // Indices of layout edges that run over a certain element
        VertexEdgeAttribute<std::set<pm::edge_index>> covered(t_m);

        std::set<pm::edge_index> conflicting_l_e;

        std::set<pm::edge_index> blocked_l_e;

        for (const auto& l_e : l_m.edges()) {
            if (embedded_l_e.count(l_e)) {
                continue;
            }

            const int l_v_a_id = l_e.vertexA().idx.value;
            const int l_v_b_id = l_e.vertexB().idx.value;
            if (l_v_components.equivalent(l_v_a_id, l_v_b_id)) {
                blocked_l_e.insert(l_e);
                continue;
            }

            const auto& path = find_shortest_path(_em, l_e);
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

        std::cout << "Embedded:        " << embedded_l_e.size() << std::endl;
        std::cout << "Blocked:         " << blocked_l_e.size() << std::endl;
        std::cout << "Conflicting:     " << conflicting_l_e.size() << std::endl;
        std::cout << "Non-conflicting: " << non_conflicting_l_e.size() << std::endl;

        if (conflicting_l_e.empty()) {
            break;
        }

        num_potential_leaves *= conflicting_l_e.size();

        const int i = std::rand() % conflicting_l_e.size();
        const auto l_e_new_idx = *std::next(conflicting_l_e.cbegin(), i);
        const auto l_e_new = l_m.edges()[l_e_new_idx];
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << "Inserting edge " << l_e_new.idx.value << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
        const auto l_he_new = l_e_new.halfedgeA();
        const auto path = find_shortest_path(_em, l_he_new);
        embed_path(_em, l_he_new, path);
        embedded_l_e.insert(l_e_new);

        const int l_v_a_id = l_e_new.vertexA().idx.value;
        const int l_v_b_id = l_e_new.vertexB().idx.value;
        l_v_components.merge(l_v_a_id, l_v_b_id);

        ++num_decisions;
    }

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << num_potential_leaves << " potential leaves" << std::endl;
    std::cout << num_decisions << " decisions" << std::endl;
}
