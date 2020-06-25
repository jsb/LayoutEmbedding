#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include "UnionFind.hh"
#include "Embedding.hh"
#include "RefinableMesh.hh"

#include "praun2001.hh"
#include "layout_generation.hh"
#include "visualization.hh"

int main()
{
    std::srand(std::time(nullptr));
    tg::rng rng;
    rng.seed(std::time(nullptr));

    glow::glfw::GlfwContext ctx;
    
    pm::Mesh t_m;
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load("/home/jsb/projects/research/topology/layout-embedding-models/hand_vulcan.obj", t_m, t_pos);

    pm::Mesh l_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    make_layout_by_decimation(t_pos, 26, l_m, l_pos);

    RefinableMesh rm = make_refinable_mesh(t_m, t_pos);
    Embedding em = make_embedding(l_m, rm);

    std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>> matching_vertices;
    for (const auto& l_v : l_m.vertices()) {
        auto best_distance_sqr = tg::inf<double>;
        auto best_t_v = pm::vertex_handle::invalid;
        for (const auto& t_v : t_m.vertices()) {
            const auto distance_sqr = tg::distance_sqr(l_pos[l_v], t_pos[t_v]);
            if (distance_sqr < best_distance_sqr) {
                best_t_v = t_v;
                best_distance_sqr = distance_sqr;
            }
        }
        matching_vertices.emplace_back(l_v, best_t_v);
    }

    set_matching_vertices(em, matching_vertices);
    praun2001(em);
    view_embedding(em);
}
