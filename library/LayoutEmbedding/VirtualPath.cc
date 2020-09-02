#include "VirtualPath.hh"

namespace LayoutEmbedding {

void set_mesh(VirtualPath& _path, const pm::Mesh& _m)
{
    for (auto& vv : _path) {
        set_mesh(vv, _m);
    }
}

VirtualPath on_mesh(const VirtualPath& _path, const pm::Mesh& _m)
{
    auto result = _path;
    set_mesh(result, _m);
    return result;
}

}
