#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/timing/CpuTimer.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <BranchAndBound.hh>
#include <Embedding.hh>
#include <LayoutGeneration.hh>
#include <Praun2001.hh>
#include <RefinableMesh.hh>
#include <Visualization/Visualization.hh>

#include <algorithm>
#include <numeric>

int main()
{
    glow::glfw::GlfwContext ctx;

    const std::string data_path = LAYOUT_EMBEDDING_DATA_PATH;
    
    // Load Target Mesh from file
    pm::Mesh t_m;
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/target-meshes/horse_8078.obj", t_m, t_pos);

    // Load Layout Mesh from file
    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/layouts/horse_layout_praun_challenge.obj", l_m, l_pos);

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

    std::cout << "Layout Mesh: ";
    std::cout << l_m.vertices().count() << " vertices, ";
    std::cout << l_m.edges().count() << " edges, ";
    std::cout << l_m.faces().count() << " faces. ";
    std::cout << "χ = " << pm::euler_characteristic(l_m) << std::endl;

    std::cout << "Target Mesh: ";
    std::cout << t_m.vertices().count() << " vertices, ";
    std::cout << t_m.edges().count() << " edges, ";
    std::cout << t_m.faces().count() << " faces. ";
    std::cout << "χ = " << pm::euler_characteristic(t_m) << std::endl;

    // Wrap the Target Mesh into a RefinableMesh to enable adaptive refinement
    RefinableMesh rm = make_refinable_mesh(t_m, t_pos);

    // Create the embedding linking the Layout and the (wrapped) Target Mesh
    Embedding em = make_embedding(l_m, rm);

    // Find embedding positions for the Layout vertices on the Target Mesh (nearest neighbors)
    auto matching_vertices = find_matching_vertices(l_pos, t_pos);

    // Optional: Perturb the target positions on the surface of the Target Mesh a bit
    jitter_matching_vertices(l_m, t_m, matching_vertices, 10);

    // Store the vertex embedding positions
    set_matching_vertices(em, matching_vertices);

    glow::timing::CpuTimer timer;

    branch_and_bound(em);

    // Run the algorithm in "Consistent Mesh Parameterizations" (Praun et al. 2001) to find embeddings for the layout edges
    /*
    Praun2001Settings settings;
    settings.insertion_order = Praun2001Settings::InsertionOrder::BestFirst;
    settings.use_swirl_detection = true;
    praun2001(em, settings);
    */

    // Visualize the result
    std::cout << "Optimization took " << timer.elapsedSeconds() << " s" << std::endl;
    std::cout << "Total embedding length: " << total_embedded_path_length(em) << std::endl;
    view_embedding(em);
}
