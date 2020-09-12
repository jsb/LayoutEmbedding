#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

namespace
{

std::pair<Embedding, pm::halfedge_attribute<tg::dpos2>> parametrize(EmbeddingInput& _input)
{
    // Compute embedding
    Embedding em(_input);
    embed_greedy(em);
    em = smooth_paths(em);

    // Compute integer-grid map
    auto l_subdivisions = choose_loop_subdivisions(em, 0.05);
    auto param = parametrize_patches(em, l_subdivisions);

    return std::make_pair(em, param);
}

}

int main()
{
    glow::glfw::GlfwContext ctx;

    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/hand_TMBF_003_v3.obj";
    const auto dir = fs::path("/servers/ash/hdd-storage1/pschmidt/Backups/SurfaceMapsMeshes/TMBF_hands/");
    auto texture = read_texture(fs::path(LE_DATA_PATH) / "textures/param_blue.png");

    {
        auto g = gv::grid();
        auto style = default_style();

        // Load input files
        std::vector<std::string> filenames { "001.obj", "002.obj", "003.obj", "004.obj", "005.obj", "006.obj" };
        std::vector<EmbeddingInput> inputs(filenames.size());
        for (int i = 0; i < filenames.size(); ++i)
        {
            inputs[i].load(layout_path, dir / filenames[i]);
            inputs[i].l_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });
            inputs[i].t_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });
        }

        // Transfer layout vertex positions from 003.obj to all others
        const auto& input_source = inputs[2];
        for (int i = 0; i < inputs.size(); ++i)
        {
//            if (i != 2)
//                continue;

            if (&inputs[i] != &input_source)
            {
                for (auto l_v : inputs[i].l_m.vertices())
                {
                    const auto l_v_source = input_source.l_m.vertices()[l_v.idx];
                    const auto t_v_source = input_source.l_matching_vertex[l_v_source];
                    const auto t_v = inputs[i].t_m.vertices()[t_v_source];

                    inputs[i].l_matching_vertex[l_v] = t_v;
                    inputs[i].l_pos[l_v] = inputs[i].t_pos[t_v];
                }
            }

            // Compute
            auto [em, param] = parametrize(inputs[i]);

            // View checkerboard
            {
                auto v = gv::view(em.target_pos(), gv::textured(param.map([] (auto p) { return tg::pos2(p.x, p.y); }), texture));
                view_vertices_and_paths(em);
            }
        }
    }
}
