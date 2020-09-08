#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/InsertionSequence.hh>

namespace LayoutEmbedding {

struct GreedySettings
{
    enum class InsertionOrder
    {
        BestFirst,
        Arbitrary,
    };
    InsertionOrder insertion_order = InsertionOrder::BestFirst;

    // Try to detect swirled paths and postpone their embedding [Praun2001]
    bool use_swirl_detection = false;
    double swirl_penalty_factor = 2.0;

    // Use path tracing using a harmonic field that tries to avoid layout vertices [Praun2001]
    bool use_vertex_repulsive_tracing = false;

    // Prefer insertion of edges that connect extremal vertices (with large average distance to neighbors) [Schreiner2004]
    bool prefer_extremal_vertices = false;
    double extremal_vertex_ratio = 0.25;
};

struct GreedyResult
{
    GreedyResult() = default;

    GreedyResult(const std::string& _algorithm, const GreedySettings& _settings)
        : algorithm(_algorithm), settings(_settings) { }

    std::string algorithm;
    GreedySettings settings;
    InsertionSequence insertion_sequence;
    double cost = std::numeric_limits<double>::infinity();
};

// Run a single greedy variant
GreedyResult embed_greedy(Embedding& _em, const GreedySettings& _settings = GreedySettings(), const std::string& _name = "greedy");
GreedyResult embed_praun(Embedding& _em, const GreedySettings& _settings = GreedySettings());
GreedyResult embed_schreiner(Embedding& _em, const GreedySettings& _settings = GreedySettings());

// Run multiple greedy variants
std::vector<GreedyResult> embed_greedy(Embedding& _em, const std::vector<GreedySettings>& _all_settings);
std::vector<GreedyResult> embed_competitors(Embedding& _em, const GreedySettings& _settings = GreedySettings());
std::vector<GreedyResult> embed_greedy_brute_force(Embedding& _em, const GreedySettings& _settings = GreedySettings());

const GreedyResult& best(const std::vector<GreedyResult>& _results);
const GreedyResult& best(const std::vector<GreedyResult>& _results, int& best_idx);

}
