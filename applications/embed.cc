#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

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

    // View embedding
    {
        auto style = default_style();
        view_embedding(em);
    }
}
