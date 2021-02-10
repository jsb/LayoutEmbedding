/**
  * Command line interface to our algorithm.
  */

#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <cxxopts.hpp>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;

int main(int argc, char** argv)
{
    register_segfault_handler();

    fs::path layout_path;
    fs::path target_path;
    std::string algo = "bnb";
    bool smooth = false;
    bool open_viewer = false;

    cxxopts::Options opts("embed",
        "Embeds a given layout into a target mesh.\n"
        "Layout connectivity is provided as a polygon mesh.\n"
        "Layout vertices are projected to target surface to define landmark positions.\n"
        "\n"
        "Output files are written to <build-folder>/output/embed.\n"
        "\n"
        "Supported algorithms are:\n"
        "    bnb:       Branch-and-bound algorithm (default)\n"
        "    greedy:    Greedy algorithm, always choosing shortest path\n"
        "    praun:     Greedy algorithm with heuristic based on [Praun2001]\n"
        "    kraevoy:   Greedy algorithm with heuristic based on [Kraevoy2003] / [Kraevoy2004]\n"
        "    schreiner: Greedy algorithm with heuristic based on [Schreiner2004]\n");
    opts.add_options()("l,layout", "Path to layout mesh.", cxxopts::value<std::string>());
    opts.add_options()("t,target", "Path to target mesh. Must be a triangle mesh.", cxxopts::value<std::string>());
    opts.add_options()("a,algo", "Algorithm, one of: bnb, greedy, praun, kraevoy, schreiner.", cxxopts::value<std::string>()->default_value("bnb"));
    opts.add_options()("s,smooth", "Apply smoothing post-process based on [Praun2001].", cxxopts::value<bool>());
    opts.add_options()("v,viewer", "Open a window to inspect the resulting embedding.", cxxopts::value<bool>());
    opts.add_options()("h,help", "Help.");
    opts.parse_positional({"layout", "target"});
    opts.positional_help("[layout] [target]");
    opts.show_positional_help();
    try {
        auto args = opts.parse(argc, argv);

        layout_path = args["layout"].as<std::string>();
        target_path = args["target"].as<std::string>();

        algo = args["algo"].as<std::string>();
        const std::set<std::string> valid_algos = { "bnb", "greedy", "praun", "kraevoy", "schreiner" };
        if (valid_algos.count(algo) == 0) {
            throw cxxopts::OptionException("Invalid algo: " + algo);
        }

        smooth = args["smooth"].as<bool>();
        open_viewer = args["viewer"].as<bool>();

        if (args.count("help") || args.count("layout") == 0 || args.count("target") == 0) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
    }
    catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    // Load input
    EmbeddingInput input;
    input.load(layout_path, target_path);

    // Compute embedding
    Embedding em(input);
    if (algo == "greedy")
        embed_greedy(em);
    else if (algo == "praun")
        embed_praun(em);
    else if (algo == "kraevoy")
        embed_kraevoy(em);
    else if (algo == "schreiner")
        embed_schreiner(em);
    else if (algo == "bnb")
        branch_and_bound(em);
    else
        LE_ASSERT(false);

    // Smooth embedding
    if (smooth)
        em = smooth_paths(em);

    // Save embedding
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "embed";
    fs::create_directories(output_dir);
    em.save(output_dir / target_path.stem());

    // Save screenshot
    {
        auto style = default_style();
        const auto screenshot_path = output_dir / (target_path.stem().string() + ".png");
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_embedding(em);
    }

    // View embedding
    if (open_viewer) {
        auto style = default_style();
        view_embedding(em);
    }
}
