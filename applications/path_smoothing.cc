/**
  * Demonstrates path smoothing based on [Praun2001]
  */

#include <glow-extras/glfw/GlfwContext.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;
    glow::glfw::GlfwContext ctx;

    // Load layout embedding from file
    EmbeddingInput input;
    input.load(fs::path(LE_DATA_PATH) / "models/layouts/horse_layout.obj",
               fs::path(LE_DATA_PATH) / "models/target-meshes/horse_8078.obj");

    Embedding em(input);

    BranchAndBoundSettings settings;
    settings.use_greedy_init = false;
    settings.optimality_gap = 0.05;
    branch_and_bound(em, settings);

    // Save embedding
    const auto embeddings_dir = fs::path(LE_OUTPUT_PATH) / "path_smoothing/embeddings";
    fs::create_directories(embeddings_dir);
    em.save(embeddings_dir / "before_smoothing");

    {
        auto g = gv::grid();
        auto style = default_style();

        view_target(em);

        em = smooth_paths(em);
        em.save(embeddings_dir / "after_smoothing");

        view_target(em);
    }
}
