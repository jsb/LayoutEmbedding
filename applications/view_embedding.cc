/**
  * View a saved embedding file.
  *
  * Usage
  *     ./view_embedding <path_to_lem_file>
  *
  */

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/StackTrace.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    EmbeddingInput input;
    Embedding em(input);

    // Load Embedding from file
    if (argc > 1) {
        const fs::path filename = argv[1];
        if (!em.load(filename.parent_path() / filename.stem())) {
            std::cout << "Embedding file could not be loaded." << std::endl;
            return 1;
        }

        auto style = default_style();
        view_embedding(em);
    }
    else {
        std::cout << "Please specify a path to a .lem file." << std::endl;
    }
}
