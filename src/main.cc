#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <Embedding.hh>
#include <LayoutGeneration.hh>
#include <Praun2001.hh>
#include <RefinableMesh.hh>
#include <Visualization/Visualization.hh>

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
    load(data_path + "/models/layouts/horse_layout.obj", l_m, l_pos);

    // Generate Layout from the Target Mesh by incremental decimation
    /*
    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    make_layout_by_decimation(t_pos, 28, l_m, l_pos);
    */

    // Wrap the Target Mesh into a RefinableMesh to enable adaptive refinement
    RefinableMesh rm = make_refinable_mesh(t_m, t_pos);

    // Create the embedding linking the Layout and the (wrapped) Target Mesh
    Embedding em = make_embedding(l_m, rm);

    // Find embedding positions for the Layout vertices on the Target Mesh (nearest neighbors)
    auto matching_vertices = find_matching_vertices(l_pos, t_pos);

    // Optional: Perturb the target positions on the surface of the Target Mesh a bit
    // jitter_matching_vertices(l_m, t_m, matching_vertices, 1);

    // Store the vertex embedding positions
    set_matching_vertices(em, matching_vertices);

    // Run the algorithm in "Consistent Mesh Parameterizations" (Praun et al. 2001) to find embeddings for the layout edges
    Praun2001Settings settings;
    settings.insertion_order = Praun2001Settings::InsertionOrder::Arbitrary;
    settings.use_swirl_detection = false;
    praun2001(em, settings);

    // Visualize the result
    view_embedding(em);
}
