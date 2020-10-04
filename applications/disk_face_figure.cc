const bool open_viewer = true;
/**
  * Embeds a disk topology layout into a disk topology target surface.
  *
  * Output files can be found in <build-folder>/output/disk_figure.
  *
  */

#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080) * 1.5;
const int screenshot_samples = 64;

void project_to_plane(
        pm::vertex_attribute<tg::pos3>& pos)
{
    const auto n = pos.mesh().faces().avg([&] (auto f) { return pm::face_normal(f, pos); });
    const auto b1 = tg::normalize(tg::cross(n, tg::vec3(0, 0, 1)));
    const auto b2 = tg::normalize(tg::cross(n, b1));
    for (auto& p : pos)
    {
        const auto u = tg::dot((p - tg::pos3::zero), b1);
        const auto v = tg::dot((p - tg::pos3::zero), b2);
        p = tg::pos3::zero + u * b1 + v * b2;
    };
}

void smooth(
        pm::vertex_attribute<tg::pos3>& pos)
{
    const int iters = 5;
    const double damp = 0.125;

    for (int iter = 0; iter < iters; ++iter)
    {
        const auto pos_prev = pos;
        for (auto v : pos.mesh().vertices())
        {
//            if (v.is_boundary())
//                continue;

            const auto avg = v.adjacent_vertices().avg([&] (auto neigh) { return pos_prev[neigh]; });
            pos[v] += damp * (avg - pos[v]);
        }
    }
}

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "disk_figure";
    fs::create_directories(output_dir);

    const auto cam_pos = glow::viewer::camera_transform(tg::pos3(-0.460784f, 0.451778f, 0.941845f), tg::pos3(-0.291428f, 0.296843f, 0.611157f));

    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/human/face_disk2.obj";
    const auto target_path = fs::path(LE_DATA_PATH) / "models/target-meshes/human/face_disk.obj";

    EmbeddingInput input;
    input.load(layout_path, target_path);

    Embedding em(input);
    BranchAndBoundSettings settings;
    settings.use_greedy_init = false;
    branch_and_bound(em, settings);

    em = smooth_paths(em, 1);

    // Save embedding
    const auto embeddings_dir = output_dir / "embeddings";
    fs::create_directories(embeddings_dir);
    em.save(embeddings_dir / "face_embedding");

    // Layout screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = output_dir / "face_layout.png";
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        project_to_plane(em.layout_pos());
        smooth(em.layout_pos());
        view_layout(em, true, 5, 7, true);
    }

    // Embedding screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = output_dir / "face_embedding.png";
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_target(em, true, 5, 7);
    }

    if (open_viewer)
    {
        auto cfg_style = default_style();
        view_embedding(em);
    }
}
