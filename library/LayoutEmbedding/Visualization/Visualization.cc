#include "Visualization.hh"

#include <LayoutEmbedding/Visualization/HaltonColorGenerator.hh>
#include <LayoutEmbedding/Visualization/RWTHColorGenerator.hh>
#include <LayoutEmbedding/Snake.hh>

namespace LayoutEmbedding {

namespace
{

pm::face_attribute<tg::color3> generate_patch_colors(
        const pm::Mesh& _m,
        const double _brightness)
{
    pm::face_attribute<tg::color3> colors(_m);
    HaltonColorGenerator halton(4);
    RWTHColorGenerator rwth;

    for (const auto f : _m.faces())
    {
        if (f.idx.value < 12)
            colors[f] = rwth.generate_next_color();
        else
            colors[f] = halton.generate_next_color();

        colors[f] = tg::mix(colors[f], tg::color3::white, _brightness);
    }

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

void view_layout(
        const Embedding& _em,
        const bool patch_colors,
        const float _point_size,
        const float _line_width,
        const bool _use_layout_input_positions)
{
    const pm::Mesh& l_m = _em.layout_mesh();

    pm::vertex_attribute<tg::color3> l_v_color = make_layout_vertex_colors(_em);
    pm::edge_attribute<tg::color3> l_e_color = make_layout_edge_colors(_em);

    auto v = gv::view();

    // Mesh
    const auto l_pos = _use_layout_input_positions ?
              _em.layout_pos()
            : make_layout_mesh_positions(_em);

    if (patch_colors)
    {
        //view_layout_mesh(_em, generate_patch_colors(_em.layout_mesh()).map([] (auto c) { return tg::color4(c.r, c.g, c.b, 0.5); }));
        gv::view(l_pos, generate_patch_colors(_em.layout_mesh(), 0.5)/*, gv::no_shading*/);
    }
    else
        view_layout(_em);

    // Embedded layout edges
    for (const auto l_e : l_m.edges()) {
        view_edge(l_pos, l_e, l_e_color[l_e], _line_width);
    }

    // Layout nodes
    for (const auto l_v : l_m.vertices()) {
        view_vertex(l_pos, l_v, l_v_color[l_v], _point_size);
    }
}

void view_target(
        const Embedding& _em,
        const bool patch_colors,
        const float _point_size,
        const float _line_width)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    auto v = gv::view();

    // Mesh
    if (patch_colors && _em.is_complete()) {
        const auto l_f_colors = generate_patch_colors(_em.layout_mesh(), 0.5);
        auto t_f_colors = _em.target_mesh().faces().make_attribute<tg::color3>(tg::color3::white);
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

    view_vertices_and_paths(_em, true, _point_size, _line_width);
}

void view_vertices_and_paths(
        const Embedding& _em,
        const bool _paths,
        const float _point_size,
        const float _line_width)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    pm::vertex_attribute<tg::color3> l_v_color = make_layout_vertex_colors(_em);
    pm::edge_attribute<tg::color3> l_e_color = make_layout_edge_colors(_em);

    auto v = gv::view();

    // Embedded layout edges
    if (_paths)
    {
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
        gv::view(gv::lines(path_pos).line_width_px(_line_width), path_color, gv::no_shading, gv::maybe_empty);
    }

    // Layout nodes
    for (const auto l_v : l_m.vertices()) {
        const auto& t_v = _em.matching_target_vertex(l_v);
        view_vertex(t_pos, t_v, l_v_color[l_v], _point_size);
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
//    gv::view(gv::lines(t_pos).line_width_px(1.0f), tg::color3{0.1f, 0.1f, 0.1f});
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

pm::vertex_attribute<tg::color3> make_layout_vertex_colors(const Embedding& _em)
{
    auto result = _em.layout_mesh().vertices().make_attribute<tg::color3>();
    HaltonColorGenerator color_generator;
    for (const auto l_v : _em.layout_mesh().vertices()) {
        result[l_v] = color_generator.generate_next_color();
    }
    return result;
}

pm::edge_attribute<tg::color3> make_layout_edge_colors(const Embedding& _em)
{
    auto result = _em.layout_mesh().edges().make_attribute<tg::color3>();
    HaltonColorGenerator color_generator;
    for (const auto l_e : _em.layout_mesh().edges()) {
        result[l_e] = color_generator.generate_next_color();
    }
    return result;
}

void view_path(const Embedding& _em, const std::vector<pm::vertex_handle>& _path, const tg::color3& _color, float _width)
{
    const auto& t_pos = _em.target_pos();
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& p_i = t_pos[_path[i]];
        const auto& p_j = t_pos[_path[i+1]];
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(_width), _color, gv::no_shading);
}

void view_path(const Embedding& _em, const VirtualPath& _path, const tg::color3& _color, float _width)
{
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& p_i = _em.element_pos(_path[i]);
        const auto& p_j = _em.element_pos(_path[i+1]);
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(_width), _color, gv::no_shading);
}

void view_path(const Embedding& _em, const Snake& _snake, const tg::color3& _color, float _width)
{
    std::vector<tg::segment3> path_segments;
    for (int i = 0; i < _snake.vertices.size() - 1; ++i) {
        const auto& p_i = _snake.vertices[i].point(_em.target_pos());
        const auto& p_j = _snake.vertices[i+1].point(_em.target_pos());
        path_segments.push_back({p_i, p_j});
    }
    gv::view(glow::viewer::lines(path_segments).line_width_px(_width), _color, gv::no_shading);
}

void view_virtual_paths(const Embedding& _em, const pm::edge_attribute<VirtualPath>& _virtual_paths, float _width)
{
    auto l_e_color = make_layout_edge_colors(_em);
    for (const auto l_e : _em.layout_mesh().edges()) {
        const auto& vp = _virtual_paths[l_e];
        if (vp.empty()) {
            continue;
        }
        view_path(_em, vp, l_e_color[l_e], _width);
    }
}

void view_edge(const pm::vertex_attribute<tg::pos3>& _pos, const pm::edge_handle& _e, const tg::color3& _color, float _width)
{
    const auto& p_i = _pos[_e.vertexA()];
    const auto& p_j = _pos[_e.vertexB()];
    gv::view(gv::lines(tg::segment3{p_i, p_j}).line_width_px(_width), _color, gv::no_shading);
}

void view_vertex(const pm::vertex_attribute<tg::pos3>& _pos, const pm::vertex_handle& _v, const tg::color3& _color, float _size)
{
    const auto& p = _pos[_v];
    gv::view(glow::viewer::points(p).point_size_px(_size), _color, gv::no_shading);
}

void view_vertex(const pm::vertex_attribute<tg::dpos3>& _pos, const pm::vertex_handle& _v, const tg::color3& _color, float _size)
{
    view_vertex(_pos.map([] (auto p) { return tg::pos3(p[0], p[1], p[2]); }), _v, _color, _size);
}

void view_param(const VertexParam& _param)
{
    gv::view(gv::lines(_param.map([] (auto p) { return tg::pos3(p.x, p.y, 0.0); })));
}

void view_param(const std::vector<pm::face_handle>& _fs, const HalfedgeParam& _param)
{
    auto seg = [&] (pm::halfedge_handle h)
    {
        const auto from = _param[h];
        const auto to = _param[h.next()];

        return tg::segment3(tg::pos3(from.x, from.y, 0.0), tg::pos3(to.x, to.y, 0.0));
    };

    std::vector<tg::segment3> segments;
    for (auto f : _fs)
    {
        for (auto h : f.halfedges())
            segments.push_back(seg(h));
    }

    gv::view(gv::lines(segments));
}

void view_quad_mesh(const pm::vertex_attribute<tg::pos3>& _q_pos, const pm::face_attribute<pm::face_handle>& _q_matching_layout_face)
{
    const auto l_colors = generate_patch_colors(*_q_matching_layout_face.first().mesh, 0.3);
    const auto q_colors = _q_pos.mesh().faces().map([&] (auto q_f) { return l_colors[_q_matching_layout_face[q_f]]; });

    auto v = gv::view(_q_pos, q_colors, /*gv::no_shading,*/ gv::ssao_power(1));
    gv::view(gv::lines(_q_pos).line_width_px(1));
}

glow::SharedTexture2D read_texture(const fs::path &_file_path)
{
    return glow::Texture2D::createFromFile(_file_path, glow::ColorSpace::sRGB);
}

}
