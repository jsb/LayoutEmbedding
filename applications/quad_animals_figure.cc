#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/QuadMeshing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;

const auto output_dir = fs::path(LE_OUTPUT_PATH) / "quad_animals";
const auto input_dir = fs::path(LE_DATA_PATH) / "models";
const auto layout_rest_path = input_dir / "layouts/box_animal_layout.obj";

namespace
{

void quad(
        const fs::path& _layout_path,
        const fs::path& _target_path,
        const glow::viewer::camera_transform& _cam_pos)
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
//        view_quad_mesh(q_pos, q_matching_layout_face, 6);
//        view_quad_mesh(q_pos, q_matching_layout_face, 11);
//        view_quad_mesh(q_pos, q_matching_layout_face, 251);
//        view_quad_mesh(q_pos, q_matching_layout_face, 256);
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
        auto cfg_view = gv::config(_cam_pos);
//        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, (output_dir / _target_path.stem()).string() + "_layout.png", GL_RGBA8));
        view_layout(em_rest, true, default_point_size, default_line_width, true);
    }
}

}

int main()
{
    glow::glfw::GlfwContext ctx;
    fs::create_directories(output_dir);

    quad(input_dir / "layouts/horse_layout.obj",
         input_dir / "target-meshes/horse_8078.obj",
         glow::viewer::camera_transform(tg::pos3(1.867239f, 0.896712f, 0.666981f), tg::pos3(0.137403f, 0.053328f, 0.117139f)));

    quad(input_dir / "layouts/quad_animals/giraffe.obj",
         input_dir / "target-meshes/quad_animals/giraffe.obj",
         glow::viewer::camera_transform(tg::pos3(1.386245f, 0.558812f, 0.649098f), tg::pos3(0.936166f, 0.374987f, 0.453374f)));

    quad(input_dir / "layouts/quad_animals/pig.obj",
         input_dir / "target-meshes/quad_animals/pig.obj",
         glow::viewer::camera_transform(tg::pos3(0.741345f, 0.500829f, -1.333263f), tg::pos3(0.573432f, 0.380441f, -1.032823f)));
}
