/**
  * Embeds a cube layout on a sphere using random landmark positions.
  * Writes lower and upper bound events to file.
  *
  * Output files can be found in <build-folder>/output/optimization_timeline.
  *
  */

#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path optimization_timeline_output_dir = output_dir / "optimization_timeline";

    const int seed = 281; // Produces a visually interesting sequence of upper and lower bound events.
    std::cout << "Seed: " << seed << std::endl;
    const fs::path cached_embedding_path = optimization_timeline_output_dir / (std::to_string(seed) + ".lem");
    const fs::path cached_embedding_path_without_extension = optimization_timeline_output_dir / std::to_string(seed);

    EmbeddingInput input;
    load(data_path / "models/target-meshes/sphere.obj", input.t_m, input.t_pos);
    load(data_path / "models/layouts/cube_layout.obj", input.l_m, input.l_pos);
    std::srand(seed);
    randomize_matching_vertices(input);

    Embedding em(input);
    if (fs::exists(cached_embedding_path)) {
        std::cout << "Loading cached embedding " << cached_embedding_path << " ..." << std::endl;
        em.load(cached_embedding_path_without_extension);
    }
    else {
        std::cout << "Computing embedding ..." << std::endl;

        BranchAndBoundSettings settings;
        settings.use_greedy_init = false;
        settings.record_lower_bound_events = true;
        settings.record_upper_bound_events = true;
        settings.time_limit = 5 * 60;
        auto result = branch_and_bound(em, settings);

        const fs::path lb_path = optimization_timeline_output_dir / (std::to_string(seed) + "_lower_bound.csv");
        const fs::path ub_path = optimization_timeline_output_dir / (std::to_string(seed) + "_upper_bound.csv");

        fs::create_directories(optimization_timeline_output_dir);
        {
            std::ofstream f{lb_path};
            f << "t,lower_bound" << '\n';
            for (const auto& record : result.lower_bound_events) {
                f << record.t << "," << record.lower_bound << '\n';
            }
        }
        {
            std::ofstream f{ub_path};
            f << "t,upper_bound" << '\n';
            for (const auto& record : result.upper_bound_events) {
                f << record.t << "," << record.upper_bound << '\n';
            }
        }

        std::cout << "Saving embedding " << cached_embedding_path << " ..." << std::endl;
        em.save(cached_embedding_path_without_extension);
    }

    // Layout screenshot
    {
        auto cfg_style = default_style();
        const auto cam_pos = glow::viewer::camera_transform(tg::pos3(-2.750460f, 2.037175f, 4.701994f), tg::pos3(-2.184680f, 1.625002f, 3.783660f));
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = optimization_timeline_output_dir / (std::to_string(seed) + "_layout.png");
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_layout(em, true, 12.0, 7.0, true);
    }

    // Embedding screenshot
    em = smooth_paths(em, 2);
    {
        auto cfg_style = default_style();
        const auto cam_pos = glow::viewer::camera_transform(tg::pos3(-0.403092f, 0.426785f, 1.924197f), tg::pos3(-0.282427f, 0.295313f, 1.375118f));
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = optimization_timeline_output_dir / (std::to_string(seed) + "_embedding.png");
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_target(em, true, 12.0, 7.0);
    }
}
