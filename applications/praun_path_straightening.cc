#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;

namespace
{

void straighten_embedded_paths(
        const std::filesystem::path& _path_prefix)
{
    namespace fs = std::filesystem;

    // Load layout embedding from file
    EmbeddingInput input;
    Embedding em(input);
    LE_ASSERT(em.load_embedding(_path_prefix));

    {
        auto v = gv::view();
        auto style = default_style();
        view_embedding(em);
    }
}

}

int main()
{
    namespace fs = std::filesystem;
    glow::glfw::GlfwContext ctx;

    { // SHREC
        const auto dir = fs::path(LE_OUTPUT_PATH) / "shrec07_results" / "saved_embeddings";

        straighten_embedded_paths(dir / "384_bnb"); // Wolf
    }
}
