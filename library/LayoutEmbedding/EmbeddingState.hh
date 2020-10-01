#pragma once

#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Hash.hh>
#include <LayoutEmbedding/InsertionSequence.hh>

namespace LayoutEmbedding {

/// EmbeddingState wraps a (copy of an) Embedding and provides additional functionality to
/// - modify its local Embedding by inserting additional edges as shortest paths,
/// - keep track of the cost (total length) of inserted edges,
/// - compute candidate paths for further insertions, determine their cost and conflicts among them.
struct EmbeddingState
{
    explicit EmbeddingState(const Embedding& _em, const BranchAndBoundSettings& _settings);
    explicit EmbeddingState(const EmbeddingState& _es) = default;

    void extend(const pm::edge_index& _l_ei, const VirtualPath& _path);

    void compute_candidate_path(const pm::edge_index& _l_ei);
    void compute_all_candidate_paths();
    void detect_candidate_path_conflicts();

    std::vector<pm::edge_index> get_conflicting_candidates(const pm::edge_index& _l_ei);

    bool valid() const;
    double cost_lower_bound() const;
    double embedded_cost() const;
    double unembedded_cost() const;

    HashValue hash() const;

    Embedding em;
    InsertionSequence insertion_sequence;

    std::set<pm::edge_index> embedded_edges() const;
    std::set<pm::edge_index> unembedded_edges() const;
    std::set<pm::edge_index> conflicting_edges() const;
    std::set<pm::edge_index> non_conflicting_edges() const;

    pm::edge_attribute<VirtualPath> candidate_paths;
    std::set<std::pair<pm::edge_index, pm::edge_index>> conflicts;

    const BranchAndBoundSettings* settings;
};

}
