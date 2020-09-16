#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

namespace
{

void parametrize(EmbeddingInput& _input)
{
    // Compute embedding
    Embedding em(_input);
    embed_greedy(em);
    em = smooth_paths(em);

    // Compute integer-grid map
    auto l_subdivisions = choose_loop_subdivisions(em, 0.05);
    auto param = parametrize_patches(em, l_subdivisions);

    // Extract quad mesh
    pm::Mesh q;
    pm::face_attribute<pm::face_handle> q_matching_layout_face;
    auto q_pos = extract_quad_mesh(em, param, q, q_matching_layout_face);

    // View
    {
        auto g = gv::grid();
        auto style = default_style();

        { // Checkerboard
            auto texture = read_texture(fs::path(LE_DATA_PATH) / "textures/param_blue.png");
            auto v = gv::view(em.target_pos(), gv::textured(param.map([] (auto p) { return 0.5 * tg::pos2(p.x, p.y); }), texture));
            view_vertices_and_paths(em);
        }

        {
            auto v = gv::view();
            view_quad_mesh(q_pos, q_matching_layout_face);
//            view_vertices_and_paths(em);
//            view_layout(em);
        }
    }
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

        parametrize(input);
    }

}
