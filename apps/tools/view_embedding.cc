/**
  * View a saved embedding file.
  */

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Util/StackTrace.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <cxxopts.hpp>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    register_segfault_handler();

    fs::path embedding_path;

    cxxopts::Options opts("view_embedding",
        "View a saved embedding file (.lem).\n");
    opts.add_options()("e,embedding", "Path to embedding file (.lem).", cxxopts::value<std::string>());
    opts.add_options()("h,help", "Help.");
    opts.parse_positional({"embedding"});
    opts.positional_help("[embedding]");
    opts.show_positional_help();
    try {
        auto args = opts.parse(argc, argv);
        if (args.count("help")) {
            std::cout << opts.help() << std::endl;
            return 0;
        }

        embedding_path = args["embedding"].as<std::string>();
    }
    catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    EmbeddingInput input;
    Embedding em(input);

    // Load Embedding from file
    if (!em.load(embedding_path.parent_path() / embedding_path.stem())) {
        std::cout << "Embedding file could not be loaded." << std::endl;
        return 1;
    }

    auto style = default_style();
    view_embedding(em);
}
