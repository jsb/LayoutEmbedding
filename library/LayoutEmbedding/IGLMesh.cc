#include "IGLMesh.hh"

#include <LayoutEmbedding/Util/Assert.hh>

namespace LayoutEmbedding {

IGLMesh to_igl_mesh(
        const pm::vertex_attribute<tg::pos3>& _pos)
{
    const pm::Mesh& m = _pos.mesh();

    LE_ASSERT(m.is_compact());
    const int num_v = m.vertices().size();
    const int num_f = m.faces().size();

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

void to_polymesh(
        const IGLMesh& _igl,
        pm::Mesh& _m,
        pm::vertex_attribute<tg::dpos3>& _pos)
{
    _m.clear();
    _pos = _m.vertices().make_attribute<tg::dpos3>();

    // Add vertices
    for (int i = 0; i < _igl.V.rows(); ++i) {
        const auto v = _m.vertices().add();
        if (_igl.V.cols() == 2)
            _pos[v] = tg::dpos3(_igl.V(i, 0), _igl.V(i, 1), 0.0);
        else if (_igl.V.cols() == 3)
            _pos[v] = tg::dpos3(_igl.V(i, 0), _igl.V(i, 1), _igl.V(i, 2));
        else
            LE_ERROR_THROW("");
    }

    // Add faces
    for (int i = 0; i < _igl.F.rows(); ++i) {
        _m.faces().add(_m.vertices()[_igl.F(i, 0)],
                       _m.vertices()[_igl.F(i, 1)],
                       _m.vertices()[_igl.F(i, 2)]);
    }
}

void to_polymesh_param(
        const Eigen::MatrixXd& _W,
        const pm::Mesh& _m,
        pm::vertex_attribute<tg::dpos2>& _param)
{
    _param = _m.vertices().make_attribute<tg::dpos2>();

    for (auto v : _m.vertices())
        _param[v] = tg::dpos2(_W(v.idx.value, 0), _W(v.idx.value, 1));
}

}
