#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

int main(int argc, char** argv)
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

        const auto& input_source = inputs[2];
        for (int i = 0; i < inputs.size(); ++i)
        {
//            if (i != 2)
//                continue;

            // Transfer layout vertex positions from 003.obj to all others
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

            // Compute embedding
            Embedding em(inputs[i]);

            std::string name;
            if (flag("--greedy", argc, argv))
            {
                embed_greedy(em);
                name = "greedy";
            }
            else if (flag("--praun", argc, argv))
            {
                embed_praun(em);
                name = "praun";
            }
            else if (flag("--kraevoy", argc, argv))
            {
                embed_kraevoy(em);
                name = "kraevoy";
            }
            else if (flag("--schreiner", argc, argv))
            {
                embed_schreiner(em);
                name = "schreiner";
            }
            else
            {
                BranchAndBoundSettings settings;
                settings.time_limit = 2 * 60;
                branch_and_bound(em, settings);
                name = "bnb";
            }

            // Smooth embedding
            if (flag("--smooth", argc, argv))
                em = smooth_paths(em);

            // Compute integer-grid map
            auto l_subdivisions = choose_loop_subdivisions(em, 0.05);
            auto param = parametrize_patches(em, l_subdivisions);

            // View checkerboard
            {
                auto v = gv::view(em.target_pos(), gv::textured(param.map([] (auto p) { return tg::pos2(p.x, p.y); }), texture), name);
                view_vertices_and_paths(em);
            }
        }
    }
}
