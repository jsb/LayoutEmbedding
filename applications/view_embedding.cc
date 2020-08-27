// Source file allowing for fleible testcases
#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/timing/CpuTimer.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/Praun2001.hh>
//#include <LayoutEmbedding/RefinableMesh.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>

#include <algorithm>
#include <fstream>
#include <numeric>

using namespace LayoutEmbedding;

int main(int argc, char** argv)
{
    glow::glfw::GlfwContext ctx;

    const std::string data_path = LE_DATA_PATH;

    // Default files for Target and Layout Mesh
    std::string embedding_file = data_path + "/models/target-meshes/horse_8078.obj";
    std::string layout_mesh_file = data_path + "/models/layouts/horse_layout_praun_challenge.obj";

    // Path to directory storing finished embeddings
    std::string result_dir = "./Results/";
    std::string model_name;

    if(argc > 1)
    {
        // Assuming first extra argument is model_name of embedding
        embedding_file = result_dir + std::string(argv[1]);
        std::cout << embedding_file << std::endl;
    }

    // Embedding Input
    EmbeddingInput input;
    // Create the embedding linking the Layout and the (wrapped) Target Mesh
    Embedding em(input);

    // Load Embedding from file
    if(!em.load_embedding(embedding_file))
    {
        std::cout << "Embedding File could not be loaded." << std::endl;
        return 1;
    }

    view_embedding(em);
}
