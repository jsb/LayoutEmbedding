#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(2560, 1440);
const int screenshot_samples = 64;
const auto cam_pos = glow::viewer::camera_transform(tg::pos3(3.749751f, 3.345287f, 6.143961f), tg::pos3(2.400282f, 1.918233f, 4.134966f));

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/teaser_pig.obj";
    const auto target_path = fs::path(LE_DATA_PATH) / "models/target-meshes/pig/pig_union.obj";
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "teaser_pig";
    fs::create_directories(output_dir);

    EmbeddingInput input;
    input.load(layout_path, target_path);

    std::vector<std::string> algorithms =
    {
        "greedy",
//        "praun",
//        "kraevoy",
//        "schreiner",
        "bnb",
    };

    // Layout screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = output_dir / "layout.png";
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_layout(Embedding(input));
    }

    // Target mesh screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = output_dir / "target.png";
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_target(Embedding(input));
    }

    for (const auto& algorithm : algorithms)
    {
        // Compute embedding
        Embedding em(input);
        if (algorithm == "greedy")
            embed_greedy(em);
        else if (algorithm == "praun")
            embed_praun(em);
        else if (algorithm == "kraevoy")
            embed_kraevoy(em);
        else if (algorithm == "schreiner")
            embed_schreiner(em);
        else if (algorithm == "bnb")
        {
            BranchAndBoundSettings settings;
            settings.time_limit = 1 * 60;
            branch_and_bound(em, settings);
        }

        // Smooth embedding
        em = smooth_paths(em);

        // Save embedding
        {
            const auto dir = output_dir / "embeddings";
            fs::create_directories(dir);
            em.save(dir / ("teaser_pig_" + algorithm));
        }

        // Embedding screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_pos);
            const auto screenshot_path = output_dir / (algorithm + ".png");
            GV_SCOPED_CONFIG(algorithm);
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_target(em);
        }
    }
}
