#include "visualization.hh"
#include "HaltonColorGenerator.hh"

#include <glow-extras/viewer/view.hh>

void view_embedding(const Embedding& _em)
{
    const pm::Mesh& l_m = *_em.l_m;
    const pm::Mesh& t_m = *_em.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_em.t_m->pos;

    HaltonColorGenerator color_generator;
    pm::vertex_attribute<tg::color3> l_v_color(l_m);
    pm::edge_attribute<tg::color3> l_e_color(l_m);
    for (const auto& l_v : l_m.vertices()) {
        l_v_color[l_v] = color_generator.generate_next_color();
    }
    for (const auto& l_e : l_m.edges()) {
        l_e_color[l_e] = color_generator.generate_next_color();
    }

    auto g = gv::grid();

    // Layout mesh view
    {
        pm::vertex_attribute<tg::pos3> l_pos(l_m);
        for (const auto& l_v : l_m.vertices()) {
            l_pos[l_v] = t_pos[_em.l_matching_vertex[l_v]];
        }

        // Mesh
        auto v = gv::view(l_pos, gv::no_grid, gv::no_outline);

        // Wireframe
        //gv::view(gv::lines(l_pos).line_width_px(1.0f), tg::color3{0.1f, 0.1f, 0.1f});

        // Embedded layout edges
        for (const auto& l_e : l_m.edges()) {
            if (is_embedded(_em, l_e)) {
                const auto& p_i = l_pos[l_e.vertexA()];
                const auto& p_j = l_pos[l_e.vertexB()];
                const auto& color = l_e_color[l_e];
                gv::view(lines(tg::segment3{p_i, p_j}).line_width_px(1.5f), color);
            }
        }

        // Layout nodes
        for (const auto& l_v : l_m.vertices()) {
            const auto& p = l_pos[l_v];
            const auto& color = l_v_color[l_v];
            gv::view(points(p).point_size_px(4.0f), color);
        }
    }

    // Target mesh view
    {
        // Mesh
        auto v = gv::view(t_pos, gv::no_grid, gv::no_outline);

        // Wireframe
        //gv::view(gv::lines(t_pos).line_width_px(1.0f), tg::color3{0.9f, 0.9f, 0.9f});

        // Embedded layout edges
        for (const auto& l_e : l_m.edges()) {
            if (is_embedded(_em, l_e)) {
                std::vector<pm::vertex_handle> t_v_path = get_embedded_path(_em, l_e.halfedgeA());
                std::vector<tg::segment3> path_segments;
                for (int i = 0; i < t_v_path.size() - 1; ++i) {
                    const auto& p_i = t_pos[t_v_path[i]];
                    const auto& p_j = t_pos[t_v_path[i+1]];
                    path_segments.push_back({p_i, p_j});
                }
                const auto& color = l_e_color[l_e];
                gv::view(lines(path_segments).line_width_px(1.5f), color);
            }
        }

        // Layout nodes
        for (const auto& l_v : l_m.vertices()) {
            const auto& p = t_pos[_em.l_matching_vertex[l_v]];
            const auto& color = l_v_color[l_v];
            gv::view(points(p).point_size_px(4.0f), color);
        }
    }
}
