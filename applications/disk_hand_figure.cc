const bool open_viewer = true;
/**
  * Embeds a disk topology layout into a genus 0 target surface.
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

const auto screenshot_size = tg::ivec2(1920, 1080) * 3;
const int screenshot_samples = 64;

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "disk_figure";
    fs::create_directories(output_dir);

    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/human/hand_disk.obj";
    const auto layout_rest_path = fs::path(LE_DATA_PATH) / "models/layouts/human/hand_disk_rest.obj";
    const auto target_path = fs::path(LE_DATA_PATH) / "models/target-meshes/human/kneeling_closed.obj";

    EmbeddingInput input;
    input.load(layout_path, target_path);

    Embedding em(input);
    BranchAndBoundSettings settings;
    settings.use_greedy_init = false;
    branch_and_bound(em, settings);

    em = smooth_paths(em, 2);

    // Save embedding
    const auto embeddings_dir = output_dir / "embeddings";
    fs::create_directories(embeddings_dir);
    em.save(embeddings_dir / "hand_embedding");

    const std::vector<glow::viewer::camera_transform> cam_poses =
    {
        glow::viewer::camera_transform(tg::pos3(0.052100f, 0.355749f, 0.054042f), tg::pos3(-0.100641f, 0.389507f, 0.014364f)),
        glow::viewer::camera_transform(tg::pos3(1.351546f, 0.104149f, 0.866480f), tg::pos3(0.952066f, -0.001433f, 0.617655f))
    };
    const std::vector<float> line_widths =
    {
        28,
        5,
    };

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
        auto cfg_view = gv::config(cam_poses.front(), gv::no_shadow);
        const auto screenshot_path = output_dir / ("hand_layout.png");
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_layout(em_rest, true, 0.6 * line_widths.front(), line_widths.front(), true);
    }

    for (int i = 0; i < cam_poses.size(); ++i)
    {
        // Embedding screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_poses[i]);
            const auto screenshot_path = output_dir / ("hand_embedding" + std::to_string(i) + ".png");
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_target(em, true, 0.6 * line_widths[i], line_widths[i]);
        }
    }

    if (open_viewer)
    {
        auto cfg_style = default_style();
        auto g = gv::grid();
        {
            auto no_shadow = gv::config(gv::no_shadow);
            view_layout(em);
        }
        {
            view_target(em);
        }
    }
}
