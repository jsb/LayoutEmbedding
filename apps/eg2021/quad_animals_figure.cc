/**
  * Embeds quad layout into three animals and computes quad meshes.
  *
  * If "--viewer" is enabled, multiple windows will open successively.
  * Press ESC to close the current window.
  *
  * Output files can be found in <build-folder>/output/quad_animals_figure.
  */

#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <cxxopts.hpp>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080) * 2;
const int screenshot_samples = 64;

const auto output_dir = fs::path(LE_OUTPUT_PATH) / "quad_animals_figure";
const auto input_dir = fs::path(LE_DATA_PATH) / "models";
const auto layout_rest_path = input_dir / "layouts/quad_animals/box_animal.obj";

namespace
{

void quad_greedy(
        const fs::path& _layout_path,
        const fs::path& _target_path,
        const glow::viewer::camera_transform& _cam_pos,
        bool _open_viewer)
{
    // Compute embedding
    EmbeddingInput input;
    input.load(_layout_path, _target_path);
    Embedding em(input);

    embed_greedy(em);

    em = smooth_paths(em);

    // Save embedding
    const auto embeddings_dir = output_dir / "embeddings";
    fs::create_directories(embeddings_dir);
    em.save(embeddings_dir / (_target_path.stem().string() + "_greedy"));

    // Compute integer-grid map
    auto l_subdivisions = choose_loop_subdivisions(em, 0.05);

    // Reduce subdivision in some places for clearer visualization
    const auto h_start = pm::halfedge_from_to(em.layout_mesh().vertices()[10], em.layout_mesh().vertices()[29]);
    auto h = h_start;
    do
    {
        l_subdivisions[h.edge()] = 3;
        h = h.next().next().opposite();
    }
    while (h != h_start);

    auto param = parametrize_patches(em, l_subdivisions);

    // Extract quad mesh
    pm::Mesh q;
    pm::face_attribute<pm::face_handle> q_matching_layout_face;
    auto q_pos = extract_quad_mesh(em, param, q, q_matching_layout_face);

    // Quad mesh screenshot
    {
        auto style = default_style();
        auto cfg_view = gv::config(_cam_pos);
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, (output_dir / _target_path.stem()).string() + "_greedy.png", GL_RGBA8));
        view_quad_mesh(q_pos, q_matching_layout_face);
//        view_quad_mesh(q_pos, q_matching_layout_face, 256); // Enable this and check out quad-animals branch to achieve paper style
    }

    if (_open_viewer)
    {
        auto style = default_style();
        auto g = gv::columns();
        view_layout(em);
        view_target(em);
        view_quad_mesh(q_pos, q_matching_layout_face);
    }
}

void quad_bnb(
        const fs::path& _layout_path,
        const fs::path& _target_path,
        const glow::viewer::camera_transform& _cam_pos,
        bool _open_viewer)
{
    // Compute embedding
    EmbeddingInput input;
    input.load(_layout_path, _target_path);
    Embedding em(input);

    BranchAndBoundSettings settings;
    settings.use_greedy_init = false;
    settings.optimality_gap = 0.05;
    branch_and_bound(em, settings);

    em = smooth_paths(em);

    // Save embedding
    const auto embeddings_dir = output_dir / "embeddings";
    fs::create_directories(embeddings_dir);
    em.save(embeddings_dir / (_target_path.stem().string() + "_bnb"));

    // Compute integer-grid map
    auto l_subdivisions = choose_loop_subdivisions(em, 0.05);
    auto param = parametrize_patches(em, l_subdivisions);

    // Extract quad mesh
    pm::Mesh q;
    pm::face_attribute<pm::face_handle> q_matching_layout_face;
    auto q_pos = extract_quad_mesh(em, param, q, q_matching_layout_face);

    // Quad mesh screenshot
    {
        auto style = default_style();
        auto cfg_view = gv::config(_cam_pos);
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, (output_dir / _target_path.stem()).string() + ".png", GL_RGBA8));
        view_quad_mesh(q_pos, q_matching_layout_face);
//        view_quad_mesh(q_pos, q_matching_layout_face, 256); // Enable this and check out quad-animals branch to achieve paper style
    }

    // Layout screenshot
    {
        // Load rest state vertex positions
        Embedding em_rest = em;
        pm::Mesh m_rest;
        auto pos_rest = m_rest.vertices().make_attribute<tg::pos3>();
        pm::load(layout_rest_path, m_rest, pos_rest);
        for (auto v : em_rest.layout_mesh().vertices())
            em_rest.layout_pos()[v] = pos_rest[m_rest[v.idx]];

        auto cfg_style = default_style();
        auto cfg_view = gv::config(_cam_pos, gv::no_shadow);
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, (output_dir / _target_path.stem()).string() + "_layout.png", GL_RGBA8));

        auto v = gv::view();

        gv::view(gv::lines(pos_rest).line_width_px(2.2));
        view_layout(em_rest, true, 0, 0, true);
    }

    if (_open_viewer)
    {
        auto style = default_style();
        auto g = gv::columns();
        view_layout(em);
        view_target(em);
        view_quad_mesh(q_pos, q_matching_layout_face);
    }
}

}

int main(int argc, char** argv)
{
    register_segfault_handler();

    bool open_viewer = false;
    cxxopts::Options opts("quad_animals_figure", "Generates Fig. 3");
    opts.add_options()("v,viewer", "Open viewer widget", cxxopts::value<bool>()->default_value("false"));
    opts.add_options()("h,help", "Help");
    try {
        auto args = opts.parse(argc, argv);
        if (args.count("help")) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
        open_viewer = args["viewer"].as<bool>();
    } catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    fs::create_directories(output_dir);

    quad_bnb(input_dir / "layouts/horse_layout.obj",
         input_dir / "target-meshes/horse_8078.obj",
         glow::viewer::camera_transform(tg::pos3(1.867239f, 0.896712f, 0.666981f), tg::pos3(0.137403f, 0.053328f, 0.117139f)),
         open_viewer);

    quad_bnb(input_dir / "layouts/quad_animals/pig.obj",
         input_dir / "target-meshes/quad_animals/pig.obj",
         glow::viewer::camera_transform(tg::pos3(0.741345f, 0.500829f, -1.333263f), tg::pos3(0.573432f, 0.380441f, -1.032823f)),
         open_viewer);

    quad_bnb(input_dir / "layouts/quad_animals/giraffe.obj",
         input_dir / "target-meshes/quad_animals/giraffe.obj",
         glow::viewer::camera_transform(tg::pos3(1.386245f, 0.558812f, 0.649098f), tg::pos3(0.936166f, 0.374987f, 0.453374f)),
         open_viewer);

    quad_greedy(input_dir / "layouts/quad_animals/giraffe.obj",
         input_dir / "target-meshes/quad_animals/giraffe.obj",
         glow::viewer::camera_transform(tg::pos3(1.386245f, 0.558812f, 0.649098f), tg::pos3(0.936166f, 0.374987f, 0.453374f)),
         open_viewer);
}
