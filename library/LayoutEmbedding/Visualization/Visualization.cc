#include "Visualization.hh"

#include <LayoutEmbedding/Visualization/HaltonColorGenerator.hh>
#include <LayoutEmbedding/Snake.hh>

namespace LayoutEmbedding {

void view_embedding(const Embedding& _em)
{
    auto g = gv::grid();
    {
        view_layout(_em);
    }
    {
        view_target(_em);
    }
}

void view_layout(const Embedding& _em)
{
    const pm::Mesh& l_m = _em.layout_mesh();

    HaltonColorGenerator color_generator;
    pm::vertex_attribute<tg::color3> l_v_color(l_m);
    pm::edge_attribute<tg::color3> l_e_color(l_m);
    for (const auto l_v : l_m.vertices()) {
        l_v_color[l_v] = color_generator.generate_next_color();
    }
    for (const auto l_e : l_m.edges()) {
        l_e_color[l_e] = color_generator.generate_next_color();
    }

    auto v = gv::view();

    // Mesh
    auto l_pos = make_layout_mesh_positions(_em);
    view_layout_mesh(_em);

    // Embedded layout edges
    for (const auto l_e : l_m.edges()) {
        if (_em.is_embedded(l_e)) {
            view_edge(l_pos, l_e, l_e_color[l_e]);
        }
    }

    // Layout nodes
    for (const auto l_v : l_m.vertices()) {
        view_vertex(l_pos, l_v, l_v_color[l_v]);
    }
}

void view_target(const Embedding& _em)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    HaltonColorGenerator color_generator;
    pm::vertex_attribute<tg::color3> l_v_color(l_m);
    pm::edge_attribute<tg::color3> l_e_color(l_m);
    for (const auto l_v : l_m.vertices()) {
        l_v_color[l_v] = color_generator.generate_next_color();
    }
    for (const auto l_e : l_m.edges()) {
        l_e_color[l_e] = color_generator.generate_next_color();
    }

    auto v = gv::view();

    // Mesh
    view_target_mesh(_em);

    // Embedded layout edges
    for (const auto l_e : l_m.edges()) {
        if (_em.is_embedded(l_e)) {
            std::vector<pm::vertex_handle> t_v_path = _em.get_embedded_path(l_e.halfedgeA());
            view_path(_em, t_v_path, l_e_color[l_e]);
        }
    }

    // Layout nodes
    for (const auto l_v : l_m.vertices()) {
        const auto& t_v = _em.matching_target_vertex(l_v);
        view_vertex(t_pos, t_v, l_v_color[l_v]);
    }
}

void view_layout_mesh(const Embedding& _em)
{
    // Mesh
    auto l_pos = make_layout_mesh_positions(_em);
    gv::view(l_pos);

    // Wireframe
    //gv::view(gv::lines(l_pos).line_width_px(1.0f), tg::color3{0.1f, 0.1f, 0.1f});
}

void view_target_mesh(const Embedding& _em)
{
    // Mesh
    auto t_pos = _em.target_pos();
    gv::view(t_pos);

    // Wireframe
    //gv::view(gv::lines(t_pos).line_width_px(1.0f), tg::color3{0.1f, 0.1f, 0.1f});
}

pm::vertex_attribute<tg::pos3> make_layout_mesh_positions(const Embedding& _em)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    pm::vertex_attribute<tg::pos3> l_pos(l_m);
    for (const auto& l_v : l_m.vertices()) {
        l_pos[l_v] = t_pos[_em.matching_target_vertex(l_v)];
    }
    return l_pos;
}

void view_path(const Embedding& _em, const std::vector<pm::vertex_handle>& _path, const tg::color3& _color)
{
    const float arc_width = 2.0f; // TODO: parameter?
    const auto& t_pos = _em.target_pos();
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& p_i = t_pos[_path[i]];
        const auto& p_j = t_pos[_path[i+1]];
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(arc_width), _color, gv::no_shading);
}

void view_path(const Embedding& _em, const VirtualPath& _path, const tg::color3& _color)
{
    const float arc_width = 2.0f; // TODO: parameter?
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& p_i = _em.element_pos(_path[i]);
        const auto& p_j = _em.element_pos(_path[i+1]);
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(arc_width), _color, gv::no_shading);
}

void view_path(const Embedding& _em, const Snake& _snake, const tg::color3& _color)
{
    const float arc_width = 2.0f; // TODO: parameter?
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _snake.vertices.size() - 1; ++i) {
        const auto& p_i = _snake.vertices[i].point(_em.target_pos());
        const auto& p_j = _snake.vertices[i+1].point(_em.target_pos());
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(arc_width), _color, gv::no_shading);
}

void view_edge(const pm::vertex_attribute<tg::pos3>& _pos, const pm::edge_handle& _e, const tg::color3& _color)
{
    const float arc_width = 2.0f; // TODO: parameter?
    const auto& p_i = _pos[_e.vertexA()];
    const auto& p_j = _pos[_e.vertexB()];
    gv::view(gv::lines(tg::segment3{p_i, p_j}).line_width_px(arc_width), _color, gv::no_shading);
}

void view_vertex(const pm::vertex_attribute<tg::pos3> &_pos, const polymesh::vertex_handle &_v, const tg::color3 &_color)
{
    const float node_size = 5.0f; // TODO: parameter?
    const auto& p = _pos[_v];
    gv::view(glow::viewer::points(p).point_size_px(node_size), _color, gv::no_shading);
}

void view_vertex(const pm::vertex_attribute<tg::dpos3> &_pos, const polymesh::vertex_handle &_v, const tg::color3 &_color)
{
    view_vertex(_pos.map([] (auto p) { return tg::pos3(p[0], p[1], p[2]); }), _v, _color);
}

}
