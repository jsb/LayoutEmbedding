#include "Visualization.hh"

#include <LayoutEmbedding/Visualization/HaltonColorGenerator.hh>
#include <LayoutEmbedding/Visualization/RWTHColorGenerator.hh>
#include <LayoutEmbedding/Snake.hh>

namespace LayoutEmbedding {

namespace
{

pm::face_attribute<tg::color3> generate_patch_colors(
        const pm::Mesh& _m)
{
    HaltonColorGenerator color_generator(0);
    pm::face_attribute<tg::color3> colors(_m);

    const float brightness = 0.50;

    for (const auto f : _m.faces())
        colors[f] = tg::mix(color_generator.generate_next_color(), tg::color3::white, brightness);

    return colors;
}

}

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

void view_layout(const Embedding& _em, const bool patch_colors)
{
    const pm::Mesh& l_m = _em.layout_mesh();

    HaltonColorGenerator color_generator;
    pm::vertex_attribute<tg::color3> l_v_color(l_m);
    pm::edge_attribute<tg::color3> l_e_color(l_m);

    for (const auto l_v : l_m.vertices())
        l_v_color[l_v] = color_generator.generate_next_color();
    for (const auto l_e : l_m.edges())
        l_e_color[l_e] = color_generator.generate_next_color();

    auto v = gv::view();

    // Mesh
    auto l_pos = make_layout_mesh_positions(_em);

    if (patch_colors)
        view_layout_mesh(_em, generate_patch_colors(_em.layout_mesh()));
    else
        view_layout(_em);

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

void view_target(const Embedding& _em, const bool patch_colors)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    auto v = gv::view();

    // Mesh
    if (patch_colors && _em.is_complete()) {
        const auto l_f_colors = generate_patch_colors(_em.layout_mesh());
        auto t_f_colors = _em.target_mesh().faces().make_attribute<tg::color3>();
        for (auto l_f : _em.layout_mesh().faces()) {
            for (auto t_f : _em.get_patch(l_f)) {
                t_f_colors[t_f] = l_f_colors[l_f];
            }
        }
        view_target_mesh(_em, t_f_colors);
    }
    else {
        view_target_mesh(_em);
    }

    view_vertices_and_paths(_em);
}

void view_vertices_and_paths(const Embedding& _em)
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

    // Embedded layout edges
    {
        const float arc_width = 2.5f; // TODO: parameter?
        pm::Mesh path_mesh;
        auto path_pos = path_mesh.vertices().make_attribute<tg::pos3>();
        auto path_color = path_mesh.edges().make_attribute<tg::color3>();
        for (const auto t_e : t_pos.mesh().edges()) {
            const auto l_h = _em.matching_layout_halfedge(t_e.halfedgeA());
            if (l_h.is_invalid()) {
                continue;
            }

            const auto v1 = path_mesh.vertices().add();
            const auto v2 = path_mesh.vertices().add();
            const auto e = path_mesh.edges().add_or_get(v1, v2);
            path_pos[v1] = t_pos[t_e.vertexA()];
            path_pos[v2] = t_pos[t_e.vertexB()];
            path_color[e] = l_e_color[l_h.edge()];
        }
        gv::view(gv::lines(path_pos).line_width_px(arc_width), path_color, gv::no_shading);
    }

    // Layout nodes
    for (const auto l_v : l_m.vertices()) {
        const auto& t_v = _em.matching_target_vertex(l_v);
        view_vertex(t_pos, t_v, l_v_color[l_v]);
    }
}

void view_layout_mesh(const Embedding& _em)
{
    auto l_f_color = _em.layout_mesh().faces().make_attribute(tg::color3::white);
    view_layout_mesh(_em, l_f_color);
}

void view_layout_mesh(const Embedding& _em, const pm::face_attribute<tg::color3>& _l_f_color)
{
    auto l_pos = make_layout_mesh_positions(_em);
    gv::view(l_pos, _l_f_color);

    // Wireframe
    //gv::view(gv::lines(l_pos).line_width_px(1.0f), tg::color3{0.1f, 0.1f, 0.1f});
}

void view_target_mesh(const Embedding& _em)
{
    auto t_f_color = _em.target_mesh().faces().make_attribute(tg::color3::white);
    view_target_mesh(_em, t_f_color);
}

void view_target_mesh(const Embedding& _em, const pm::face_attribute<tg::color3>& _t_f_color)
{
    // Mesh
    auto t_pos = _em.target_pos();
    gv::view(t_pos, _t_f_color);

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
    const float arc_width = 2.5f; // TODO: parameter?
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
    const float arc_width = 2.5f; // TODO: parameter?
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
    const float arc_width = 2.5f; // TODO: parameter?
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

void view_param(const pm::vertex_attribute<tg::dpos2> _param)
{
    gv::view(gv::lines(_param.map([] (auto p) { return tg::pos3(p.x, p.y, 0.0); })));
}

glow::SharedTexture2D read_texture(const fs::path &_file_path)
{
    return glow::Texture2D::createFromFile(_file_path, glow::ColorSpace::sRGB);
}

}
