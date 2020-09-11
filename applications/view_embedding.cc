#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/StackTrace.hh>

#include <tinyfiledialogs.h>

using namespace LayoutEmbedding;

int main(int argc, char** argv)
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    EmbeddingInput input;
    Embedding em(input);

    // Load Embedding from file
    if(argc > 1) {
        const std::string embedding_file = std::string(argv[1]);
        if(!em.load_embedding(embedding_file)) {
            std::cout << "Embedding File could not be loaded." << std::endl;
            return 1;
        }

        auto style = default_style();
        view_embedding(em);
    }
    else {
        std::cout << "Please specify an embedding file." << std::endl;
    }
}
