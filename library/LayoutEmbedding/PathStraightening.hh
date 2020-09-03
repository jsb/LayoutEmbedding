#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding
{

/**
 * Returns a new Embedding instance, in which
 * embedded paths have been straightened via
 * a harmonic parametrization of the two adjacent
 * patches, as described in [Praun2001].
 */
Embedding straighten_paths(
        const Embedding& _em_orig,
        const int _n_iters = 1);

}
