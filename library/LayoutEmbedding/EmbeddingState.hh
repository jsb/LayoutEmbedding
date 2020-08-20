#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Hash.hh>

#include <vector>

namespace LayoutEmbedding {

using InsertionSequence = std::vector<pm::edge_index>;

/// EmbeddingState wraps a (copy of an) Embedding and provides additional functionality to
/// - modify its local Embedding by inserting additional edges as shortest paths,
/// - keep track of the cost (total length) of inserted edges,
/// - compute candidate paths for further insertions, determine their cost and conflicts among them.
struct EmbeddingState
{
    struct CandidatePath
    {
        VirtualPath path;
        double cost = std::numeric_limits<double>::infinity();
    };

    explicit EmbeddingState(const Embedding& _em);
    explicit EmbeddingState(const EmbeddingState& _es) = default;

    void extend(const pm::edge_handle& _l_e);
    void extend(const pm::edge_index& _l_ei);
    void extend(const InsertionSequence& _seq);

    void compute_candidate_paths();

    double cost_lower_bound() const;

    HashValue hash() const;

    Embedding em;
    bool valid = true;

    double embedded_cost = 0.0;
    double unembedded_cost = 0.0;

    std::set<pm::edge_index> embedded_l_edges;
    std::set<pm::edge_index> conflicting_l_edges;
    std::set<pm::edge_index> non_conflicting_l_edges;

    pm::edge_attribute<CandidatePath> candidate_paths;
};

}
