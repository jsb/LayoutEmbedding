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

    bool use_swirl_detection = false;
    double swirl_penalty_factor = 2.0; // This value is a guess.

    bool use_vertex_repulsive_tracing = false;

    bool prefer_extremal_vertices = false;
    double extremal_vertex_ratio = 0.25;
};

struct Praun2001Result
{
    InsertionSequence insertion_sequence;
};

Praun2001Result praun2001(Embedding& _em, const Praun2001Settings& _settings = Praun2001Settings());

}
