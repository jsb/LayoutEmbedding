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
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>

#include <algorithm>
#include <fstream>
#include <numeric>

using namespace LayoutEmbedding;

int main()
{
    glow::glfw::GlfwContext ctx;
    const std::string data_path = LE_DATA_PATH;

    pm::Mesh input_t_m;
    auto input_t_pos = input_t_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/target-meshes/sphere.obj", input_t_m, input_t_pos);

    pm::Mesh input_l_m;
    auto input_l_pos = input_l_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/layouts/cube_layout.obj", input_l_m, input_l_pos);

    Embedding em(input_l_m, input_t_m, input_t_pos);
    const auto& l_m = em.layout_mesh();
    const auto& t_m = em.target_mesh();
    const auto& t_pos = em.target_pos();

    // Generate random matching vertices
    std::srand(0);
    std::vector<pm::vertex_handle> t_vertices = t_m.vertices().to_vector();
    std::random_shuffle(t_vertices.begin(), t_vertices.end());
    MatchingVertices matching_vertices;
    for (int i = 0; i < l_m.vertices().size(); ++i) {
        matching_vertices.push_back({l_m.vertices()[i], t_vertices[i]});
    }

    em.set_matching_vertices(matching_vertices);

    branch_and_bound(em);
    //praun2001(em);

    std::cout << "Embedding cost: " << em.total_embedded_path_length() << std::endl;

    auto cfg_style = gv::config(gv::no_grid, gv::no_outline, gv::background_color(RWTH_WHITE));
    auto g = gv::grid();
    view_layout(em);
    view_target(em);
}
