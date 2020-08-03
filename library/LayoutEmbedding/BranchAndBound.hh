#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding {

struct BranchAndBoundSettings
{
    double optimality_gap = 0.03;
    double timeout = 1 * 60 * 60; // Seconds. Set to <= 0 to disable.
};

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings());

}
