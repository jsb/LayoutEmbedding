#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/InsertionSequence.hh>

namespace LayoutEmbedding {

struct BranchAndBoundSettings
{
    double optimality_gap = 0.01;
    double time_limit = 1 * 60 * 60; // Seconds. Set to <= 0 to disable.

    bool record_upper_bound_events = true;
    bool record_lower_bound_events = false;

    enum class Priority
    {
        LowerBoundNonConflicting, // Priority: lower_bound * num_non_conflicting_paths
        LowerBound,               // Priority: lower_bound
    };
    Priority priority = Priority::LowerBoundNonConflicting;

    double lower_bound_priority_time_limit = 0.0; // Seconds. Set to <= 0 to disable.
    // After this time, the solver will switch to a different queue priority that focuses on improving the lower bound.

    bool print_current_insertion_sequence = true;
    bool print_memory_footprint_estimate = true;
};

struct BranchAndBoundResult
{
    BranchAndBoundResult() = default;

    BranchAndBoundResult(const std::string& _algorithm, const BranchAndBoundSettings& _settings) :
        algorithm(_algorithm),
        settings(_settings)
    {
    }

    std::string algorithm;
    BranchAndBoundSettings settings;
    InsertionSequence insertion_sequence;

    double cost = std::numeric_limits<double>::infinity();
    double lower_bound = std::numeric_limits<double>::infinity();
    double gap = 1.0;

    struct UpperBoundEvent
    {
        double t;
        double upper_bound;
    };
    std::vector<UpperBoundEvent> upper_bound_events;

    struct LowerBoundEvent
    {
        double t;
        double lower_bound;
    };
    std::vector<LowerBoundEvent> lower_bound_events;
};

BranchAndBoundResult branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings(), const std::string& _name = "bnb");

}
