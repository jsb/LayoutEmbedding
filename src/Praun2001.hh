#pragma once

#include "Embedding.hh"

struct Praun2001Settings
{
    enum class InsertionOrder
    {
        BestFirst,
        Arbitrary,
    };

    InsertionOrder insertion_order = InsertionOrder::BestFirst;
    bool use_swirl_detection = true;
};

void praun2001(Embedding& _em, const Praun2001Settings& _settings = Praun2001Settings());
