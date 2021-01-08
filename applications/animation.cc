#include <LayoutEmbedding/Util/StackTrace.hh>
#include <LayoutEmbedding/Util/Video.hh>
#include <LayoutEmbedding/Visualization/UniformBSpline.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <glow-extras/glfw/GlfwContext.hh>

#include <filesystem>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    EmbeddingInput input;
    Embedding em(input);
    em.load(fs::path(LE_OUTPUT_PATH) / "teaser_pig/embeddings/teaser_pig_greedy");


    const fs::path output_path = fs::path(LE_OUTPUT_PATH) / "animation";
    fs::create_directories(output_path);

    BSplineAnimation<glow::viewer::camera_transform, 3> bsa{
        glow::viewer::camera_transform(tg::pos3(1.019982f, 0.234125f, -0.701132f), tg::pos3(0.075501f, -0.035154f, -0.000174f)),
        glow::viewer::camera_transform(tg::pos3(-0.563370f, 0.508743f, -0.867306f), tg::pos3(0.075501f, -0.035154f, -0.000174f)),
        glow::viewer::camera_transform(tg::pos3(-1.013569f, 0.078649f, 0.506653f), tg::pos3(0.075501f, -0.035154f, -0.000174f)),
        glow::viewer::camera_transform(tg::pos3(0.635647f, 0.134537f, 1.054975f), tg::pos3(0.075501f, -0.035154f, -0.000174f)),
        glow::viewer::camera_transform(tg::pos3(1.142403f, 0.245085f, 0.488788f), tg::pos3(0.075501f, -0.035154f, -0.000174f)),
    };

    const auto screenshot_size = tg::ivec2(1920, 1080);
    const int screenshot_samples = 64;

    for (const auto& [i, t] : bsa.frames(61)) {
        const fs::path screenshot_path = output_path / (format_frame_number(i) + ".png");

        auto cfg_style = default_style();
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGB8));
        auto cfg_camera = gv::config(bsa.eval(t));

        view_target(em);
    }
    render_video(output_path, 30);
}
