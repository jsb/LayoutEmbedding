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

static bool screenshots_only = true;
static auto screenshot_size = tg::ivec2(1920, 1080);
static int screenshot_samples = 64;

int main()
{
    glow::glfw::GlfwContext ctx;
    const std::string data_path = LE_DATA_PATH;

    pm::Mesh t_m;
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/target-meshes/sphere.obj", t_m, t_pos);

    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/layouts/cube_layout.obj", l_m, l_pos);

    // Generate random matching vertices
    std::srand(0);
    std::vector<pm::vertex_handle> t_vertices = t_m.vertices().to_vector();
    std::random_shuffle(t_vertices.begin(), t_vertices.end());
    MatchingVertices matching_vertices;
    for (int i = 0; i < l_m.vertices().size(); ++i) {
        matching_vertices.push_back({l_m.vertices()[i], t_vertices[i]});
    }

    Embedding em = make_embedding(l_m, t_m, t_pos);
    set_matching_vertices(em, matching_vertices);

    branch_and_bound(em);
    //praun2001(em);

    auto cfg_style = gv::config(gv::no_grid, gv::no_outline, gv::background_color(RWTH_WHITE));
    auto g = gv::grid();
    view_layout(em);
    view_target(em);
}
