const bool open_viewer = true;
/**
  * Embeds quad layout into hands in different poses using challenging landmark positions
  * and computes quad meshes.
  *
  * If "open_viewer" is enabled, multiple windows will open successively.
  * Press ESC to close the current window.
  *
  * Output files can be found in <build-folder>/output/quad_hands.
  *
  */

#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/StackTrace.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

struct TestCase
{
    std::string filename;
    glow::viewer::camera_transform camera;
};

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    std::vector<TestCase> tests =
    {
        {
            "001.obj",
            glow::viewer::camera_transform(tg::pos3(0.329532f, 0.222535f, -1.284020f), tg::pos3(0.218050f, 0.131482f, -0.893754f)),
        },
        {
            "002.obj",
            glow::viewer::camera_transform(tg::pos3(-0.108347f, 0.312487f, -1.304698f), tg::pos3(-0.113235f, 0.193670f, -0.899082f)),
        },
        {
            "003.obj",
            glow::viewer::camera_transform(tg::pos3(-0.001297f, 0.234221f, -1.339844f), tg::pos3(-0.014470f, 0.128625f, -0.906745f)),
        },
    };

    std::vector<std::string> algorithms =
    {
        "kraevoy",
        "schreiner",
        "bnb",
    };

    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/hand_TMBF_003.obj";
    const auto input_dir = fs::path(LE_DATA_PATH) / "models/target-meshes/TMBF_hands/";
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "quad_hands";
    fs::create_directories(output_dir);

    const auto screenshot_size = tg::ivec2(2560, 1440);
    const int screenshot_samples = 64;

    // Load embedding for which vertex positions have been set
    EmbeddingInput input_ref;
    input_ref.load(layout_path, input_dir / "003.obj");
    input_ref.l_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });
    input_ref.t_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });

    // Screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(tests.front().camera);

        { // View layout
            const fs::path screenshot_path = output_dir / ("layout.png");
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_layout(Embedding(input_ref));
        }
    }

    for (const auto& test : tests)
    {
        // Load meshes
        EmbeddingInput input;
        input.load(layout_path, input_dir / test.filename);
        input.l_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });
        input.t_pos.apply([] (auto& p) { p = tg::rotate_z(p, tg::angle::from_degree(-90)); });

        // Transfer layout vertex positions from reference
        for (auto l_v : input.l_m.vertices())
        {
            const auto l_v_source = input_ref.l_m.vertices()[l_v.idx];
            const auto t_v_source = input_ref.l_matching_vertex[l_v_source];
            const auto t_v = input.t_m.vertices()[t_v_source];

            input.l_matching_vertex[l_v] = t_v;
            input.l_pos[l_v] = input.t_pos[t_v];
        }

        // Screenshots
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(test.camera);

            { // View target mesh
                const fs::path screenshot_path = output_dir / (test.filename + "_target.png");
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                view_target(Embedding(input));
            }
        }

        for (const auto& algorithm : algorithms)
        {
            // Compute embedding
            Embedding em(input);
            if (algorithm == "kraevoy")
                embed_kraevoy(em);
            else if (algorithm == "schreiner")
                embed_schreiner(em);
            else if (algorithm == "bnb")
            {
                BranchAndBoundSettings settings;
                settings.time_limit = 60;
                settings.optimality_gap = 0.02;
                branch_and_bound(em, settings);
            }
            else
                LE_ERROR_THROW("");

            // Smooth embedding
            em = smooth_paths(em);

            // Save embedding
            {
                const auto dir = output_dir / "embeddings";
                fs::create_directories(dir);
                em.save(dir / (test.filename + "_" + algorithm));
            }

            // Compute integer-grid map
            const auto l_subdivisions = choose_loop_subdivisions(em, 0.05, 13);
            const auto param = parametrize_patches(em, l_subdivisions);

            // Extract quad mesh
            pm::Mesh q;
            pm::face_attribute<pm::face_handle> q_matching_layout_face;
            const auto q_pos = extract_quad_mesh(em, param, q, q_matching_layout_face);

            // Screenshots
            {
                auto cfg_style = default_style();
                auto cfg_view = gv::config(test.camera);

                { // View embedding
                    const fs::path screenshot_path = output_dir / (test.filename + "_" + algorithm + "_embedding.png");
                    auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                    view_target(em);
                }

                { // View quad mesh
                    const fs::path screenshot_path = output_dir / (test.filename + "_" + algorithm + "_quad.png");
                    auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                    view_quad_mesh(q_pos, q_matching_layout_face);
                }
            }

            if (open_viewer)
            {
                auto style = default_style();
                auto g = gv::columns();
                view_layout(em);
                {
                    auto v = gv::view();
                    caption(algorithm);
                    view_target(em);
                }
                view_quad_mesh(q_pos, q_matching_layout_face);
            }
        }
    }
}
