#pragma once

#include <LayoutEmbedding/VirtualVertex.hh>

namespace LayoutEmbedding {

using VirtualPath = std::vector<VirtualVertex>;

// Re-setting the mesh pointer
void set_mesh(VirtualPath& _path, const pm::Mesh& _m);
VirtualPath on_mesh(const VirtualPath& _path, const pm::Mesh& _m);

}
