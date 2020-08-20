#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/InsertionSequence.hh>

namespace LayoutEmbedding {

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

struct Praun2001Result
{
    InsertionSequence insertion_sequence;
};

Praun2001Result praun2001(Embedding& _em, const Praun2001Settings& _settings = Praun2001Settings());

}
