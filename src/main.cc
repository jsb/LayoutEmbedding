#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include "UnionFind.hh"
#include "RefinableMesh.hh"

int main()
{
    std::srand(std::time(nullptr));
    tg::rng rng;

    glow::glfw::GlfwContext ctx;
    
    pm::Mesh l_m;
    pm::Mesh t_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load("/home/jsb/projects/research/topology/layout-embedding-models/horse_layout.obj", l_m, l_pos);
    load("/home/jsb/projects/research/topology/layout-embedding-models/horse_8078.obj", t_m, t_pos);

    std::cout << "n_vertices: " << t_m.vertices().count() << std::endl;
    RefinableMesh rm = make_refinable_mesh(t_m, t_pos);
    for (int i = 0; i < 100000; ++i) {
        const auto& e = t_m.edges().random(rng);
        split_edge(rm, e);
    }
    std::cout << "n_vertices: " << t_m.vertices().count() << std::endl;
    cleanup(rm);
    //t_m.compactify();
    std::cout << "n_vertices: " << t_m.vertices().count() << std::endl;

    auto l_vertex_embedding = l_m.vertices().make_attribute<pm::vertex_handle>();
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
        l_vertex_embedding[l_v] = best_t_v;
    }

    auto l_edges_shuffled = l_m.edges().to_vector();
    std::random_shuffle(begin(l_edges_shuffled), end(l_edges_shuffled));
    UnionFind uf(l_m.vertices().count());
    auto l_on_cut_graph = l_m.edges().make_attribute<bool>(false);
    for (const auto& l_e : l_edges_shuffled) {
        const auto iA = static_cast<int>(l_e.vertexA());
        const auto iB = static_cast<int>(l_e.vertexB());
        if (!uf.equivalent(iA, iB)) {
            l_on_cut_graph[l_e] = true;
            uf.merge(iA, iB);
        }
    }

    auto t_embedded_cut_graph = t_m.edges().make_attribute<pm::edge_handle>();
    
    if (false)
    {
        auto v = gv::view(l_pos, gv::no_grid, gv::no_outline);

        std::vector<tg::segment3> cut_edges;
        std::vector<tg::segment3> non_cut_edges;
        for (const auto& l_e : l_m.edges()) {
            if (l_on_cut_graph[l_e]) {
                cut_edges.push_back({l_pos[l_e.vertexA()], l_pos[l_e.vertexB()]});
            }
            else {
                non_cut_edges.push_back({l_pos[l_e.vertexA()], l_pos[l_e.vertexB()]});
            }
        }
        gv::view(lines(non_cut_edges).line_width_px(1.0f), tg::color3{0.75f, 0.75f, 0.75f});
        gv::view(lines(cut_edges).line_width_px(2.0f), tg::color3{1.0f, 0.0f, 0.0f});
        gv::view(points(l_pos).point_size_px(4.0f), tg::color3{0.0f, 0.75f, 0.0f});
    }

    {
        auto v = gv::view(t_pos, gv::no_grid, gv::no_outline);
        gv::view(gv::lines(t_pos).line_width_px(1.0f), tg::color3{0.0f, 0.0f, 0.0f});

        std::vector<tg::segment3> orig_edges;
        for (const auto& t_e : t_m.edges()) {
            if (is_on_original_mesh(rm, t_e)) {
                orig_edges.push_back({t_pos[t_e.vertexA()], t_pos[t_e.vertexB()]});
            }
        }
        gv::view(lines(orig_edges).line_width_px(1.1f), tg::color3{1.0f, 0.0f, 0.0f});

        std::vector<tg::pos3> orig_vertices;
        for (const auto& t_v : t_m.vertices()) {
            if (is_original_vertex(rm, t_v)) {
                orig_vertices.push_back(t_pos[t_v]);
            }
        }
        gv::view(points(orig_vertices).point_size_px(3.5f), tg::color3{1.0f, 0.0f, 0.0f});
    }

    for (const auto& f : t_m.faces()) {
        if (f.vertices().count() != 3) {
            std::cout << f.idx.value << std::endl;
        }
    }
}
