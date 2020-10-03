#include <string>
std::string open_prefix = "284";
/**
  * View results of the evaluation on the SHREC07 dataset.
  *
  * Instructions:
  *
  *     * Run shrec07_generate_layouts before running this file.
  *     * Run shrec07_embed_layouts before running this file.
  *
  *     * Navigate through results using left and right arrow keys.
  *     * Mesh id and embedding algorithm are shown in bottom left corner.
  *     * Close window by pressing ESC
  *     * Set "open_prefix" to start at a specific mesh id.
  *
  */

#include "shrec07.hh"
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <GLFW/glfw3.h>

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

int main()
{
    register_segfault_handler();
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
    int i_embedding = 0;
    for (int i = 0; i < embedding_files.size(); ++i)
    {
        if (starts_with(embedding_files[i].filename(), open_prefix))
        {
            i_embedding = i;
            break;
        }
    }

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
