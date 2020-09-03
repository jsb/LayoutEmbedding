#include "BranchAndBound.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>
#include <LayoutEmbedding/EmbeddingState.hh>
#include <LayoutEmbedding/Greedy.hh>

#include <chrono>
#include <queue>

namespace LayoutEmbedding {

struct State
{
    HashValue parent;
    pm::edge_index l_e;
    VirtualPath path;
};

struct Candidate
{
    double lower_bound = std::numeric_limits<double>::infinity();
    double priority = 0.0;

    //InsertionSequence insertions;
    HashValue state;

    bool operator<(const Candidate& _rhs) const
    {
        return priority > _rhs.priority;
    }
};

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings)
{
    InsertionSequence best_insertion_sequence;
    double global_upper_bound = std::numeric_limits<double>::infinity();

    // Run heuristic algorithm to find a tighter initial upper bound.
    {
        Embedding em(_em);
        auto result = embed_greedy_brute_force(em);
        global_upper_bound = em.total_embedded_path_length();
        best_insertion_sequence = result.insertion_sequence;
    }

    //std::set<HashValue> known_state_hashes;
    std::map<HashValue, State> known_states;
    {
        State root;
        root.parent = 0;
        known_states[0] = root;
    }

    // Init priority queue with empty state.
    std::priority_queue<Candidate> q;
    {
        Candidate c;
        c.lower_bound = 0.0;
        c.priority = 0.0;
        c.state = 0;
        q.push(c);
    }

    // TODO: Use glow-extras timer instead?
    const auto start_time = std::chrono::steady_clock::now();

    while (!q.empty()) {
        // Time limit
        const auto current_time = std::chrono::steady_clock::now();
        const auto interval = current_time - start_time;
        using seconds_double = std::chrono::duration<double>;
        const double elapsed_seconds = std::chrono::duration_cast<seconds_double>(interval).count();
        if (_settings.time_limit > 0.0) {
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

        // Reconstruct the embedding sequence and inserted paths by traversing the state graph
        InsertionSequence insertion_sequence;
        std::vector<const VirtualPath*> inserted_paths;
        HashValue current_state_hash = c.state;
        while (current_state_hash != 0) {
            LE_ASSERT(known_states.count(current_state_hash) > 0);
            const State& state = known_states[current_state_hash];
            insertion_sequence.push_back(state.l_e);
            inserted_paths.push_back(&state.path);
            current_state_hash = state.parent;
        }
        std::reverse(insertion_sequence.begin(), insertion_sequence.end());
        std::reverse(inserted_paths.begin(), inserted_paths.end());

        // Reconstruct the embedding associated with this state
        EmbeddingState es(_em);
        LE_ASSERT(insertion_sequence.size() == inserted_paths.size());
        for (size_t i = 0; i < insertion_sequence.size(); ++i) {
            const pm::edge_index& l_e = insertion_sequence[i];
            const VirtualPath path = on_mesh(*inserted_paths[i], es.em.target_mesh());
            es.extend(l_e, path);
        }

        LE_ASSERT(es.hash() == c.state);

        es.compute_candidate_paths();

        if (!es.valid) {
            // The current embedding might be invalid if paths run into dead ends.
            // We ignore such states.
            continue;
        }

        if (c.lower_bound > 0) {
            LE_ASSERT(es.cost_lower_bound() == c.lower_bound);
        }

        std::cout << "t: " << elapsed_seconds;
        std::cout << "    ";
        std::cout << "|Embd|: " << es.embedded_l_edges.size();
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
        std::cout << "    ";
        std::cout << "|H|: " << known_states.size();
        std::cout << "    ";
        std::cout << "|CC|: " << es.count_connected_components();
        std::cout << std::endl;

        if (es.cost_lower_bound() < global_upper_bound) {
            // Completed layout?
            if (es.conflicting_l_edges.empty()) {
                global_upper_bound = es.cost_lower_bound();
                best_insertion_sequence = insertion_sequence;
                std::cout << "New upper bound: " << global_upper_bound << std::endl;
            }
            else {
                // Add children to the queue
                for (const auto& l_e : es.conflicting_l_edges) {
                    EmbeddingState new_es(es); // Copy
                    State new_state;
                    new_state.parent = c.state;
                    new_state.l_e = l_e;
                    new_state.path = es.candidate_paths[l_e].path;
                    set_mesh(new_state.path, new_es.em.target_mesh());

                    new_es.extend(l_e, new_state.path);

                    const HashValue new_es_hash = new_es.hash();
                    const auto [it, inserted] = known_states.emplace(new_es_hash, new_state);
                    if (!inserted) {
                        //std::cout << "Skipping child state (known hash: " << new_es_hash << ")" << std::endl;
                        continue;
                    }

                    new_es.compute_candidate_paths();

                    const double new_lower_bound = new_es.cost_lower_bound();
                    const double new_gap = 1.0 - new_lower_bound / global_upper_bound;
                    if (new_gap > _settings.optimality_gap) {
                        Candidate new_c;
                        new_c.state = new_es_hash;
                        new_c.lower_bound = new_lower_bound;
                        new_c.priority = new_c.lower_bound * new_es.conflicting_l_edges.size();
                        q.push(new_c);
                    }
                }
            }
        }
    }
    std::cout << "Branch-and-bound optimization completed." << std::endl;

    {
        // Drain the rest of the queue to find the maximum optimality gap
        auto largest_gap = _settings.optimality_gap;
        while (!q.empty()) {
            auto c = q.top();
            const double gap = 1.0 - c.lower_bound / global_upper_bound;
            largest_gap = std::max(largest_gap, gap);
            q.pop();
        }
        std::cout << "The optimal solution is at most " << (largest_gap * 100.0) << " % better than the found solution." << std::endl;
    }

    // Apply the victorious embedding sequence to the input embedding

    // Edges with predefined insertion sequence
    std::set<pm::edge_index> l_e_embedded;
    for (const auto& l_ei : best_insertion_sequence) {
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
