#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

namespace
{

pm::halfedge_attribute<tg::pos2> projected_tex_coords(
        const pm::vertex_attribute<tg::pos3>& _pos,
        const tg::vec3 _right,
        const tg::vec3 _up)
{
    auto uvs = _pos.mesh().halfedges().make_attribute<tg::pos2>();
    for (auto h : _pos.mesh().halfedges())
        uvs[h] = tg::pos2(tg::dot(_pos[h.vertex_to()], _right), tg::dot(_pos[h.vertex_to()], _up));

    return uvs;
}

void show_ism(
        const fs::path& path_A,
        const fs::path& path_B)
{
    // Load overlay mesh with vertex positions on A and B
    pm::Mesh overlay_A;
    pm::Mesh overlay_B;
    auto pos_A = overlay_A.vertices().make_attribute<tg::pos3>();
    auto pos_B = overlay_B.vertices().make_attribute<tg::pos3>();
    pm::load(path_A, overlay_A, pos_A);
    pm::load(path_B, overlay_B, pos_B);

    const auto uvs_A = projected_tex_coords(pos_A, tg::vec3(0, 0, 2), tg::vec3(0, 2, 0));
    const auto texture = glow::Texture2D::createFromFile(fs::path(LE_DATA_PATH) / "textures/checkerboard.png", glow::ColorSpace::sRGB);

    // Transfer uvs
    auto uvs_B = overlay_B.halfedges().make_attribute<tg::pos2>();
    for (auto h_B : overlay_B.halfedges())
    {
        const auto h_A = pm::halfedge_from_to(overlay_A.vertices()[h_B.vertex_from().idx],
                                              overlay_A.vertices()[h_B.vertex_to().idx]);
        uvs_B[h_B] = uvs_A[h_A];
    }

    {
        auto g = gv::grid();
        auto style = default_style();
        {
            gv::view(pos_A, gv::textured(uvs_A, texture).flip());
        }
        {
            gv::view(pos_B, gv::textured(uvs_B, texture).flip());
        }
    }
}

}

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    show_ism(fs::path(LE_OUTPUT_PATH) / "inter_surface_map" / "greedy" / "AonB_overlay.obj",
             fs::path(LE_OUTPUT_PATH) / "inter_surface_map" / "greedy" / "BonA_overlay.obj");

//    show_ism(fs::path(LE_OUTPUT_PATH) / "inter_surface_map" / "bnb" / "AonB_overlay.obj",
//             fs::path(LE_OUTPUT_PATH) / "inter_surface_map" / "bnb" / "BonA_overlay.obj");
}
