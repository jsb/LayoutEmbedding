#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/InsertionSequence.hh>

namespace LayoutEmbedding {

struct BranchAndBoundSettings
{
    double optimality_gap = 0.01;
    double time_limit = 1 * 60 * 60; // Seconds. Set to <= 0 to disable.
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
};

BranchAndBoundResult branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings(), const std::string& _name = "bnb");

}
