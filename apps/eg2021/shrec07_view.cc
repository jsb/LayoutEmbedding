/**
  * View results of the evaluation on the SHREC07 dataset.
  *
  * Instructions:
  *
  *     * Run shrec07_generate_layouts before running this file.
  *     * Run shrec07_embed_layouts before running this file (~24h).
  *
  *     * Navigate through results using left and right arrow keys.
  *     * Mesh id and embedding algorithm are shown in bottom left corner.
  *     * Close window by pressing ESC
  *     * Pass a number (e.g. 284) as a command-line argument to start at a specific mesh ID.
  *
  */

#include "shrec07.hh"
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <GLFW/glfw3.h>
#include <cxxopts.hpp>

#include <string>

using namespace LayoutEmbedding;

const auto input_dir = fs::path(LE_OUTPUT_PATH) / "shrec07_results/saved_embeddings";
const auto suffix = "smoothed.lem";

namespace
{

bool starts_with(const std::string& str, const std::string& prefix)
{
    return str.size() >= prefix.size() && 0 == str.compare(0, prefix.size(), prefix);
}

bool ends_with(const std::string& str, const std::string& suffix)
{
    return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

}

int main(int argc, char** argv)
{
    register_segfault_handler();

    std::string open_prefix = "284";
    cxxopts::Options opts("shrec07_view", "Shows SHREC07 embedding results in an interactive viewer");
    opts.add_options()("i,id", "SHREC07 mesh ID to show", cxxopts::value<std::string>());
    opts.add_options()("h,help", "Help");
    opts.parse_positional({"id"});
    try {
        auto args = opts.parse(argc, argv);
        if (args.count("help")) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
        if (args.count("id")) {
            open_prefix = args["id"].as<std::string>();
        }
    } catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    // Find all files with suffix
    std::vector<fs::path> embedding_files;
    for (auto& f : fs::recursive_directory_iterator(input_dir))
    {
        if (ends_with(f.path(), suffix))
            embedding_files.push_back(f.path());
    }

    // Sort
    std::sort(embedding_files.begin(), embedding_files.end(), [](const fs::path& a, const fs::path& b) -> bool
    {
        return std::stoi(a.filename()) < std::stoi(b.filename());
    });

    // If there is a file with this prefix, open it first
    int i_embedding = -1;
    for (int i = 0; i < embedding_files.size(); ++i)
    {
        if (starts_with(embedding_files[i].filename(), open_prefix))
        {
            i_embedding = i;
            break;
        }
    }

    // If no embedding with the specified prefix can be found, open the last one.
    if (i_embedding < 0)
        i_embedding = embedding_files.size() - 1;

    // View
    do
    {
        GV_SCOPED_CONFIG(gv::close_keys(GLFW_KEY_LEFT, GLFW_KEY_RIGHT, GLFW_KEY_P));
        const auto key = gv::get_last_close_info().closed_by_key;
        if (key == GLFW_KEY_LEFT)
            i_embedding = (i_embedding + embedding_files.size() - 1) % embedding_files.size();
        else if (key == GLFW_KEY_RIGHT)
            i_embedding = (i_embedding + 1) % embedding_files.size();
        else if (key == GLFW_KEY_ESCAPE)
            break;

        EmbeddingInput input;
        Embedding em(input);
        LE_ASSERT(em.load(embedding_files[i_embedding].parent_path() / embedding_files[i_embedding].stem()));

        auto style = default_style();
        auto v = gv::view();

        // Set caption
        gv::view(gv::make_renderable(std::vector<tg::pos3>()), gv::maybe_empty, embedding_files[i_embedding].filename());
        view_target(em);
    }
    while (true);
}
