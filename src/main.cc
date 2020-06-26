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
    
    pm::Mesh t_m;
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/target-meshes/cow_6938.obj", t_m, t_pos);

    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    make_layout_by_decimation(t_pos, 28, l_m, l_pos);

    RefinableMesh rm = make_refinable_mesh(t_m, t_pos);
    Embedding em = make_embedding(l_m, rm);

    auto matching_vertices = find_matching_vertices(l_pos, t_pos);
    jitter_matching_vertices(l_m, t_m, matching_vertices, 1);
    set_matching_vertices(em, matching_vertices);
    praun2001(em);
    view_embedding(em);
}
