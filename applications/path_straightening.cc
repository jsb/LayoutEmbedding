#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/PathStraightening.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;

namespace
{

void straighten(
        const std::filesystem::path& _path_prefix)
{
    namespace fs = std::filesystem;

    // Load layout embedding from file
    EmbeddingInput input;
    Embedding em_orig(input);
    LE_ASSERT(em_orig.load_embedding(_path_prefix));

    // Straighten paths
    Embedding em_straightened = straighten_paths(em_orig);

    // Show
    {
        auto g = gv::grid();
        auto style = default_style();
        view_target(em_orig);
        view_target(em_straightened);
    }
}

}

int main()
{
    namespace fs = std::filesystem;
    glow::glfw::GlfwContext ctx;

    { // SHREC
        const auto dir = fs::path(LE_OUTPUT_PATH) / "shrec07_results" / "saved_embeddings";

        straighten(dir / "384_bnb"); // Wolf
    }
}
