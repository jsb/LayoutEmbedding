#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include "UnionFind.hh"
#include "Embedding.hh"
#include "RefinableMesh.hh"

int main()
{
    std::srand(std::time(nullptr));
    tg::rng rng;
    rng.seed(std::time(nullptr));

    glow::glfw::GlfwContext ctx;
    
    pm::Mesh l_m;
    pm::Mesh t_m;
    auto l_pos = l_m.vertices().make_attribute<tg::pos3>();
    auto t_pos = t_m.vertices().make_attribute<tg::pos3>();
    load("/home/jsb/projects/research/topology/layout-embedding-models/horse_layout.obj", l_m, l_pos);
    load("/home/jsb/projects/research/topology/layout-embedding-models/horse_8078.obj", t_m, t_pos);

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

    std::vector<tg::segment3> path_segments;
    auto l_edges_shuffled = l_m.edges().to_vector();
    std::random_shuffle(begin(l_edges_shuffled), end(l_edges_shuffled));
    UnionFind uf(l_m.vertices().count());
    auto l_on_cut_graph = l_m.edges().make_attribute<bool>(false);

    const auto embed_edge = [&](pm::edge_handle l_e) {
        auto l_he = l_e.halfedgeA();
        auto path = find_shortest_path(em, l_he);
        if (path.empty()) {
            std::cout << "oopsie" << std::endl;
            assert(false);
        }
        else {
            for (int i = 0; i < path.size() - 1; ++i) {
                int j = i + 1;
                const auto p_i = element_pos(em, path[i]);
                const auto p_j = element_pos(em, path[j]);
                path_segments.push_back({p_i, p_j});
            }
            insert_path(em, l_he, path);
        }
    };

    // Embed edges to form a spanning tree
    for (const auto& l_e : l_edges_shuffled) {
        const auto iA = static_cast<int>(l_e.vertexA());
        const auto iB = static_cast<int>(l_e.vertexB());
        if (!uf.equivalent(iA, iB)) {
            embed_edge(l_e);
            l_on_cut_graph[l_e] = true;
            uf.merge(iA, iB);
        }
    }
    // Embed remaining edges
    for (const auto& l_e : l_edges_shuffled) {
        if (!l_on_cut_graph[l_e]) {
            embed_edge(l_e);
        }
    }

    {
        auto v = gv::view(t_pos, gv::no_grid, gv::no_outline);
        gv::view(gv::lines(t_pos).line_width_px(1.0f), tg::color3{0.9f, 0.9f, 0.9f});
        gv::view(lines(path_segments).line_width_px(1.5f), tg::color3{1.0f, 0.0f, 0.0f});
    }
}
