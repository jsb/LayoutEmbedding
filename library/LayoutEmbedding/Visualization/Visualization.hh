#pragma once

#include <LayoutEmbedding/Embedding.hh>

namespace LayoutEmbedding {

/// Layout and Embedding side-by-side in a grid view
void view_embedding(const Embedding& _em);

/// Layout mesh with colorized embedded edges
void view_layout(const Embedding& _em);

/// Target mesh with colorized embedded edge paths
void view_target(const Embedding& _em);

pm::vertex_attribute<tg::pos3> make_layout_mesh_positions(const Embedding& _em);

void view_layout_mesh(const Embedding& _em);

void view_target_mesh(const Embedding& _em);

}
