#pragma once

#include <Embedding.hh>

struct BranchAndBoundSettings
{

};

void branch_and_bound(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings());

void branch_and_bound2(Embedding& _em, const BranchAndBoundSettings& _settings = BranchAndBoundSettings());
