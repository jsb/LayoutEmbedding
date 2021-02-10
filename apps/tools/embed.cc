/**
  * Command line interface to our algorithm.
  * Opens a window to inspect the resulting embedding.
  *
  * Usage:
  *     ./embed <path-to-layout> <path-to-target> <flags>
  *
  * <path-to-layout>:
  *     Layout connectivity as polygonal mesh (e.g. obj).
  *     Vertices are projected to target surface
  *     to define landmark positions.
  *
  * <path-to-target>:
  *     Target triangle mesh (e.g. obj).
  *
  * <flags>:
  *     --greedy: Run greedy algorithm, always choosing shortest path.
  *     --praun: Run greedy algorithm w/ heuristic based on [Praun2001].
  *     --kraevoy: Run greedy algorithm w/ heuristic based on [Kraevoy2003] / [Kraevoy2004].
  *     --schreiner: Run greedy algorithm w/ heuristic based on [Schreiner2004].
  *
  *     --smooth: Smoothing post-process based on [Praun2001].
  *
  *     --noview: Don't open viewer window.
  *
  * Output files can be found in <build-folder>/output/embed.
  *
  */

#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;

int main(int argc, char** argv)
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    // Load input
    LE_ASSERT(argc > 2);
    EmbeddingInput input;
    input.load(argv[1], argv[2]);

    // Compute embedding
    Embedding em(input);
    if (flag("--greedy", argc, argv))
        embed_greedy(em);
    else if (flag("--praun", argc, argv))
        embed_praun(em);
    else if (flag("--kraevoy", argc, argv))
        embed_kraevoy(em);
    else if (flag("--schreiner", argc, argv))
        embed_schreiner(em);
    else
        branch_and_bound(em);

    // Smooth embedding
    if (flag("--smooth", argc, argv))
        em = smooth_paths(em);

    // Save embedding
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "embed";
    fs::create_directories(output_dir);
    em.save(output_dir / fs::path(argv[2]).stem());

    // Save screenshot
    {
        auto style = default_style();
        const auto screenshot_path = output_dir / (fs::path(argv[2]).stem().string() + ".png");
        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
        view_embedding(em);
    }

    // View embedding
    if (!flag("--noview", argc, argv))
    {
        auto style = default_style();
        view_embedding(em);
    }
}
