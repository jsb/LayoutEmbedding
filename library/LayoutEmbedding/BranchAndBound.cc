#include "BranchAndBound.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>
#include <LayoutEmbedding/VirtualPathConflictSentinel.hh>

#include <chrono>
#include <queue>

namespace LayoutEmbedding {

using InsertionSequence = std::vector<pm::edge_index>;

struct Candidate
{
    double lower_bound = std::numeric_limits<double>::infinity();
    double priority = 0.0;
    InsertionSequence insertions;

    bool operator<(const Candidate& _rhs) const
    {
        return priority > _rhs.priority;
    }
};

struct EmbeddingState
{
    explicit EmbeddingState(const Embedding& _em) :
        em(_em),
        candidate_paths(_em.layout_mesh())
    {
    }

    explicit EmbeddingState(const EmbeddingState& _es) = default;

    Embedding em;

    bool valid = true;

    double embedded_cost = 0.0;
    double unembedded_cost = 0.0;

    std::set<pm::edge_index> embedded_l_edges;
    std::set<pm::edge_index> conflicting_l_edges;
    std::set<pm::edge_index> non_conflicting_l_edges;

    struct CandidatePath
    {
        VirtualPath path;
        double cost = std::numeric_limits<double>::infinity();
    };
    pm::edge_attribute<CandidatePath> candidate_paths;

    void extend(const pm::edge_handle& _l_e)
    {
        LE_ASSERT(_l_e.mesh == &em.layout_mesh());
        LE_ASSERT(embedded_l_edges.count(_l_e) == 0);

        auto l_he = _l_e.halfedgeA();
        auto path = em.find_shortest_path(l_he);
        if (path.empty()) {
            embedded_cost = std::numeric_limits<double>::infinity();
            valid = false;
            return;
        }
        else {
            embedded_cost += em.path_length(path);
            em.embed_path(l_he, path);
            embedded_l_edges.insert(_l_e);
        }
    }

    void extend(const pm::edge_index& _l_ei)
    {
        const auto& l_e = em.layout_mesh().edges()[_l_ei];
        extend(l_e);
    }

    void extend(const InsertionSequence& _seq)
    {
        for (const auto& l_e : _seq) {
            extend(l_e);
        }
    }

    void compute_candidate_paths()
    {
        const Embedding& c_em = em; // We don't want to modify the embedding in this method.

        LE_ASSERT(&candidate_paths.mesh() == &em.layout_mesh());
        candidate_paths.clear();
        conflicting_l_edges.clear();
        non_conflicting_l_edges.clear();
        unembedded_cost = 0.0;

        VirtualPathConflictSentinel vpcs(c_em.target_mesh());

        for (const auto& l_e : c_em.layout_mesh().edges()) {
            if (!embedded_l_edges.count(l_e)) {
                auto l_he = l_e.halfedgeA();
                auto path = c_em.find_shortest_path(l_he);
                if (path.empty()) {
                    candidate_paths[l_e].cost = std::numeric_limits<double>::infinity();
                }
                else {
                    candidate_paths[l_e].path = path;
                    candidate_paths[l_e].cost = c_em.path_length(path);
                    vpcs.insert_path(path, l_e);
                }
                unembedded_cost += candidate_paths[l_e].cost;
                // TODO: Early-out if one candidate path is invalid?
            }
        }

        conflicting_l_edges = vpcs.global_conflicts;
        for (const auto& l_e : c_em.layout_mesh().edges()) {
            if (!embedded_l_edges.count(l_e) && !conflicting_l_edges.count(l_e)) {
                non_conflicting_l_edges.insert(l_e);
            }
        }
        LE_ASSERT(embedded_l_edges.size() + conflicting_l_edges.size() + non_conflicting_l_edges.size() == em.layout_mesh().edges().size());
    }

    double cost_lower_bound() const
    {
        return embedded_cost + unembedded_cost;
    }
};

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings)
{
    Candidate best_solution; // Initially empty
    double global_upper_bound = std::numeric_limits<double>::infinity();
    // TODO: Run heuristic algorithm to find a tighter initial upper bound.

    // Init priority queue with empty state.
    std::priority_queue<Candidate> q;
    {
        Candidate c;
        c.lower_bound = 0.0;
        c.priority = 0.0;
        c.insertions = {};
        q.push(c);
    }

    // TODO: Use glow-extras timer instead?
    const auto start_time = std::chrono::steady_clock::now();

    while (!q.empty()) {
        // Time limit
        if (_settings.time_limit > 0.0) {
            const auto current_time = std::chrono::steady_clock::now();
            const auto interval = current_time - start_time;
            using seconds_double = std::chrono::duration<double>;
            const double elapsed_seconds = std::chrono::duration_cast<seconds_double>(interval).count();
            if (elapsed_seconds >= _settings.time_limit) {
                std::cout << "Reached time limit of " << _settings.time_limit << " s. Terminating." << std::endl;
                if (std::isinf(global_upper_bound)) {
                    std::cout << "Warning: No valid solution was found within that time." << std::endl;
                }
                break;
            }
        }

        auto c = q.top();
        q.pop();

        // Early-out based on lower bound cached in c.
        double gap = 1.0 - c.lower_bound / global_upper_bound;
        if (gap <= _settings.optimality_gap) {
            continue;
        }

        // Reconstruct the embedding associated with this embedding sequence
        EmbeddingState es(_em);
        es.extend(c.insertions);
        es.compute_candidate_paths();

        if (c.lower_bound > 0) {
            LE_ASSERT(es.cost_lower_bound() == c.lower_bound);
        }

        std::cout << "|Embd|: " << c.insertions.size();
        std::cout << "    ";
        std::cout << "|Conf|: " << es.conflicting_l_edges.size();
        std::cout << "    ";
        std::cout << "|Ncnf|: " << es.non_conflicting_l_edges.size();
        std::cout << "    ";
        std::cout << "LB: " << es.cost_lower_bound();
        std::cout << "    ";
        std::cout << "UB: " << global_upper_bound;
        std::cout << "    ";
        std::cout << "gap: " << (gap * 100.0) << " %";
        std::cout << "    ";
        std::cout << "|Q|: " << q.size();
        std::cout << std::endl;

        if (es.cost_lower_bound() < global_upper_bound) {
            // Completed layout?
            if (es.conflicting_l_edges.empty()) {
                global_upper_bound = es.cost_lower_bound();
                best_solution = c;
                std::cout << "New upper bound: " << global_upper_bound << std::endl;
            }
            else {
                // Add children to the queue
                for (const auto& l_e : es.conflicting_l_edges) {
                    EmbeddingState new_es(es); // Copy
                    new_es.extend(l_e);
                    new_es.compute_candidate_paths();

                    const double new_lower_bound = new_es.cost_lower_bound();
                    const double new_gap = 1.0 - new_lower_bound / global_upper_bound;
                    if (new_gap > _settings.optimality_gap) {
                        Candidate new_c;
                        new_c.insertions = c.insertions;
                        new_c.insertions.push_back(l_e);
                        new_c.lower_bound = new_lower_bound;
                        new_c.priority = new_c.lower_bound * new_es.conflicting_l_edges.size();
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
    for (const auto& l_ei : best_solution.insertions) {
        const auto l_e = _em.layout_mesh().edges()[l_ei];
        const auto l_he = l_e.halfedgeA();
        const auto path = _em.find_shortest_path(l_he);
        _em.embed_path(l_he, path);
        l_e_embedded.insert(l_e);
    }
    // Remaining edges
    for (const auto& l_e : _em.layout_mesh().edges()) {
        if (!l_e_embedded.count(l_e)) {
            const auto l_he = l_e.halfedgeA();
            const auto path = _em.find_shortest_path(l_he);
            _em.embed_path(l_he, path);
            l_e_embedded.insert(l_e);
        }
    }
}

}
