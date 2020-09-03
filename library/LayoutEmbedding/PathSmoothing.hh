#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding
{

/**
 * Returns a new Embedding instance, in which
 * embedded paths have been smoothed via straight
 * lines harmonic parametrizations of the two adjacent
 * patches, as described in [Praun2001].
 */
Embedding smooth_paths(
        const Embedding& _em_orig,
        const int _n_iters = 1);

}
