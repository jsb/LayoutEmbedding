#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding {

struct BranchAndBoundSettings
{
    double optimality_gap = 0.01;
    double time_limit = 1 * 60 * 60; // Seconds. Set to <= 0 to disable.
    bool use_hashing = true;
};

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings());

}
