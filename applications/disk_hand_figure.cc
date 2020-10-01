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
    const auto target_path = fs::path(LE_DATA_PATH) / "models/target-meshes/human/kneeling_closed.obj";

    EmbeddingInput input;
    input.load(layout_path, target_path);

    Embedding em(input);
    embed_greedy(em);

    em = smooth_paths(em, 2);

    const std::vector<glow::viewer::camera_transform> cam_poses =
    {
        glow::viewer::camera_transform(tg::pos3(0.052100f, 0.355749f, 0.054042f), tg::pos3(-0.100641f, 0.389507f, 0.014364f)),
        glow::viewer::camera_transform(tg::pos3(1.351546f, 0.104149f, 0.866480f), tg::pos3(0.952066f, -0.001433f, 0.617655f))
    };
    const std::vector<float> line_widths =
    {
        22,
        4,
    };

    for (int i = 0; i < cam_poses.size(); ++i)
    {
        // Layout screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_poses[i], gv::no_shadow);
            const auto screenshot_path = output_dir / ("hand_layout" + std::to_string(i) + ".png");
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_layout(em, true, 0.6 * line_widths[i], line_widths[i], true);
        }

        // Embedding screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_poses[i]);
            const auto screenshot_path = output_dir / ("hand_embedding" + std::to_string(i) + ".png");
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_target(em, true, 0.6 * line_widths[i], line_widths[i]);
        }
    }
}
