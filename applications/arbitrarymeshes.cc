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
    std::string target_mesh_file = data_path + "/models/target-meshes/horse_8078.obj";
    std::string layout_mesh_file = data_path + "/models/layouts/horse_layout_praun_challenge.obj";

    // Path to directory storing finished embeddings
    std::string result_dir = "./Results/";
    std::string model_name;

    if(argc > 1)
    {
        // Assuming first extra argument is Target Mesh name, adds /models/ prefix
        target_mesh_file = data_path + "/models/target-meshes/" + std::string(argv[1]);
        if(argc > 2)
        {
            // Assuming second extra argument is Layout Mesh name, adds /layouts/ prefix
            layout_mesh_file = data_path + "/models/layouts/" + std::string(argv[2]);
            model_name = std::string(argv[2]);

        }
        else
        {
            model_name = "horse_layout_praun_challenge.obj";
        }
    }

    // Embedding Input
    EmbeddingInput input;

    // Load Target Mesh from file
    pm::Mesh t_m;
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    if(!load(target_mesh_file, input.t_m, input.t_pos))
    {
        std::cout << "Target Mesh File could not be loaded." << std::endl;
        return 1;
    }

    // Load Layout Mesh from file
    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    if(!load(layout_mesh_file, input.l_m, input.l_pos))
    {
            std::cout << "Layout Mesh File could not be loaded." << std::endl;
            return 1;
    }

    // Edge permutation
    /*
    std::vector<int> p(l_m.edges().size());
    std::iota(p.begin(), p.end(), 0);
    std::srand(5);
    std::random_shuffle(p.begin(), p.end());
    l_m.edges().permute(p);
    */

    // Generate Layout from the Target Mesh by incremental decimation
    /*
    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    make_layout_by_decimation(t_pos, 28, l_m, l_pos);
    */

    // Create the embedding linking the Layout and the (wrapped) Target Mesh
    //Embedding em = make_embedding(l_m, rm);

    // Find embedding positions for the Layout vertices on the Target Mesh (nearest neighbors)
    find_matching_vertices_by_proximity(input);
    //auto matching_vertices = find_matching_vertices(l_pos, t_pos);

    // Create the embedding linking the Layout and the (wrapped) Target Mesh
    Embedding em(input);

    // Optional: Perturb the target positions on the surface of the Target Mesh a bit
    // jitter_matching_vertices(l_m, t_m, matching_vertices, 1);


    // Run the algorithm in "Consistent Mesh Parameterizations" (Praun et al. 2001) to find embeddings for the layout edges
    Praun2001Settings settings;
    settings.insertion_order = Praun2001Settings::InsertionOrder::BestFirst;
    settings.use_swirl_detection = true;
    praun2001(em, settings);

    // Visualize the result
    view_embedding(em);

    em.save(model_name);
}
