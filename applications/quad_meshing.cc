#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Harmonic.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

namespace
{

void extract_patch(
        const Embedding& _em,
        const pm::face_handle& _l_f,
        pm::Mesh& _patch,
        pm::vertex_attribute<tg::pos3>& _patch_pos,
        pm::vertex_attribute<pm::vertex_handle>& _v_target_to_patch,
        pm::halfedge_attribute<pm::halfedge_handle>& _h_patch_to_target)
{
    // Init result
    _patch.clear();
    _patch_pos = _patch.vertices().make_attribute<tg::pos3>();

    // Index maps
    _v_target_to_patch = _em.target_mesh().vertices().make_attribute<pm::vertex_handle>();
    _h_patch_to_target = _patch.halfedges().make_attribute<pm::halfedge_handle>();

    // Create region mesh
    for (auto t_f : _em.get_patch(_l_f))
    {
        // Add vertices to result mesh
        for (auto t_v : t_f.vertices())
        {
            if (_v_target_to_patch[t_v].is_invalid())
            {
                auto r_v = _patch.vertices().add();
                _patch_pos[r_v] = _em.target_pos()[t_v];
                _v_target_to_patch[t_v] = r_v;
            }
        }

        // Add face to result mesh
        _patch.faces().add(t_f.vertices().to_vector([&] (auto t_v) {
            return _v_target_to_patch[t_v];
        }));

        // Fill halfedge index map
        for (auto t_h : t_f.halfedges())
        {
            const auto r_v_from = _v_target_to_patch[t_h.vertex_from()];
            const auto r_v_to = _v_target_to_patch[t_h.vertex_to()];
            const auto r_h = pm::halfedge_from_to(r_v_from, r_v_to);
            LE_ASSERT(r_h.is_valid());

            _h_patch_to_target[r_h] = t_h;
            _h_patch_to_target[r_h.opposite()] = t_h.opposite();
        }
    }
}

auto view_param(
        const pm::vertex_attribute<tg::dpos2> _param)
{
    return gv::view(gv::lines(_param.map([] (auto p) { return tg::pos3(p.x, p.y, 0.0); })));
}

glow::SharedTexture2D read_texture(
        const fs::path &_file_path)
{
    return glow::Texture2D::createFromFile(_file_path, glow::ColorSpace::sRGB);
}

pm::halfedge_attribute<tg::dpos2> parametrize(
        const Embedding& _em)
{
    LE_ASSERT(_em.is_complete());
    auto param = _em.target_mesh().halfedges().make_attribute<tg::dpos2>();

    for (auto l_f : _em.layout_mesh().faces())
    {
        // Extract patch mesh
        pm::Mesh p_m;
        pm::vertex_attribute<tg::pos3> p_pos;
        pm::vertex_attribute<pm::vertex_handle> v_target_to_patch;
        pm::halfedge_attribute<pm::halfedge_handle> h_patch_to_target;
        extract_patch(_em, l_f, p_m, p_pos, v_target_to_patch, h_patch_to_target);

        // Constrain patch boundary
        auto p_constrained = p_m.vertices().make_attribute<bool>(false);
        auto p_constraint_value = p_m.vertices().make_attribute<tg::dpos2>();
        tg::dpos2 corner_from(0.5, -0.5);
        tg::dpos2 corner_to(0.5, 0.5);
        for (auto l_h : l_f.halfedges())
        {
            const double length_total = _em.embedded_path_length(l_h);
            double length_acc = 0.0;
            const auto t_path_vertices = _em.get_embedded_path(l_h);
            for (int i = 0; i < t_path_vertices.size() - 1; ++i)
            {
                const auto t_vi = t_path_vertices[i];
                const auto t_vj = t_path_vertices[i+1];
                const double lambda_i = length_acc / length_total;
                length_acc += tg::length(_em.target_pos()[t_vi] - _em.target_pos()[t_vj]);

                const auto p_vi = v_target_to_patch[t_vi];
                p_constrained[p_vi] = true;
                p_constraint_value[p_vi] = (1.0 - lambda_i) * corner_from + lambda_i * corner_to;
            }

            corner_from = tg::dpos2(-corner_from.y, corner_from.x);
            corner_to = tg::dpos2(-corner_to.y, corner_to.x);
        }

        // Compute Tutte embedding
        pm::vertex_attribute<tg::dpos2> p_param;
        harmonic(p_pos, p_constrained, p_constraint_value, p_param);

        // Transfer parametrization to target mesh
        for (auto p_h : p_m.halfedges())
            param[h_patch_to_target[p_h]] = p_param[p_h.vertex_to()];
    }

    return param;
}

void run(
        EmbeddingInput& _input)
{
    Embedding em(_input);
    embed_greedy(em);
    em = smooth_paths(em);

    auto param = parametrize(em);

    {   // View checkerboard
        auto texture = read_texture(fs::path(LE_DATA_PATH) / "textures/param_blue.png");
        auto v = gv::view(em.target_pos(), gv::textured(param.map([] (auto p) { return tg::pos2(p.x, p.y); }), texture));
//        view_param(param);
    }

//    {
//        auto g = gv::grid();
//        auto style = default_style();

//        view_embedding(em);
//    }
}

}

int main()
{
    glow::glfw::GlfwContext ctx;

    const auto dir = fs::path(LE_DATA_PATH) / "models";
    EmbeddingInput input;

    {   // Box animal -> SHREC
        input.load(dir / "layouts/horse_layout.obj",
                   dir / "target-meshes/horse_8078.obj");

        run(input);
    }
}
