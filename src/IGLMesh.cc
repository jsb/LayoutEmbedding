#include "IGLMesh.hh"

#include <Assert.hh>

IGLMesh to_igl_mesh(const pm::vertex_attribute<tg::pos3>& _pos)
{
    const pm::Mesh& m = _pos.mesh();

    LE_ASSERT(m.is_compact());

    const int num_v = m.vertices().count();
    const int num_f = m.faces().count();

    IGLMesh result;
    result.V.resize(num_v, 3);
    result.F.resize(num_f, 3);

    for (const auto v : m.vertices()) {
        int v_row = v.idx.value;
        result.V(v_row, 0) = _pos[v][0];
        result.V(v_row, 1) = _pos[v][1];
        result.V(v_row, 2) = _pos[v][2];
    }
    for (const auto& f : m.faces()) {
        int f_row = f.idx.value;
        int f_col = 0;
        for (const auto v : f.vertices()) {
            result.F(f_row, f_col) = v.idx.value;
            ++f_col;
        }
    }

    return result;
}
