#pragma once

#include <LayoutEmbedding/EmbeddingInput.hh>

namespace LayoutEmbedding {

/// Creates a layout mesh by decimating the target mesh down to _n_vertices.
/// The _input must contain a target mesh t_m.
/// This will overwrite the _input's layout mesh l_m.
void make_layout_by_decimation(EmbeddingInput& _input, int _n_vertices);

/// Finds a matching target mesh vertex for every layout vertex by a simple nearest neighbors search.
/// This will modify the l_matching_vertex attribute stored in _input.
void find_matching_vertices_by_proximity(EmbeddingInput& _input);

/// Modifies existing matching vertices by moving each one to a random neighboring vertex.
/// This will modify the l_matching_vertex attribute stored in _input.
void jitter_matching_vertices(EmbeddingInput& _input, int _steps = 1);

/// Sets matching vertices by assigning each layout vertex to a random target vertex.
/// This will modify the l_matching_vertex attribute stored in _input.
void randomize_matching_vertices(EmbeddingInput& _input);

}
