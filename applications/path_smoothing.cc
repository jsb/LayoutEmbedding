#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;

namespace
{

void smooth(
        const std::filesystem::path& _path_prefix)
{
    // Load layout embedding from file
    EmbeddingInput input;
    Embedding em_orig(input);
    LE_ASSERT(em_orig.load_embedding(_path_prefix));

    {
        auto g = gv::grid();
        auto style = default_style();

        // Compare results after different numbers of iterations
        for (int i = 0; i < 4; ++i)
            view_target(smooth_paths(em_orig, i));
    }
}

}

int main()
{
    namespace fs = std::filesystem;
    glow::glfw::GlfwContext ctx;

    { // SHREC
        const auto dir = fs::path(LE_OUTPUT_PATH) / "shrec07_results" / "saved_embeddings";

//        smooth(dir / "3_greedy"); // Human
        smooth(dir / "381_greedy"); // Cow
//        smooth(dir / "384_greedy"); // Wolf
    }

    { // Sphere
        const auto dir = fs::path(LE_OUTPUT_PATH) / "sphere_stress";

//        smooth(dir / "sphere_greedy");
    }
}
