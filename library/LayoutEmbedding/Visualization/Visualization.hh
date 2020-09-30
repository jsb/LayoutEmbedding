#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>
#include <LayoutEmbedding/Parametrization.hh>

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

namespace LayoutEmbedding {

constexpr float default_line_width = 5.0f;
constexpr float default_point_size = 10.0f;

/// Layout and Embedding side-by-side in a grid view
void view_embedding(const Embedding& _em);

/// Layout mesh with colorized embedded edges
void view_layout(
        const Embedding& _em,
        const bool patch_colors = true,
        const float _point_size = default_point_size,
        const float _line_width = default_line_width);

/// Target mesh with colorized embedded edge paths
void view_target(
        const Embedding& _em,
        const bool patch_colors = true,
        const float _point_size = default_point_size,
        const float _line_width = default_line_width);

void view_vertices_and_paths(
        const Embedding& _em,
        const bool _paths = true,
        const float _point_size = default_point_size,
        const float _line_width = default_line_width);

pm::vertex_attribute<tg::pos3> make_layout_mesh_positions(const Embedding& _em);
pm::vertex_attribute<tg::color3> make_layout_vertex_colors(const Embedding& _em);
pm::edge_attribute<tg::color3> make_layout_edge_colors(const Embedding& _em);

void view_layout_mesh(const Embedding& _em);
void view_layout_mesh(const Embedding& _em, const pm::face_attribute<tg::color3>& _l_f_color);

void view_target_mesh(const Embedding& _em);
void view_target_mesh(const Embedding& _em, const pm::face_attribute<tg::color3>& _t_f_color);

void view_path(const Embedding& _em, const std::vector<pm::vertex_handle>& _path, const tg::color3& _color, float _width = default_line_width);
void view_path(const Embedding& _em, const VirtualPath& _path, const tg::color3& _color, float _width = default_line_width);
void view_path(const Embedding& _em, const Snake& _snake, const tg::color3& _color, float _width = default_line_width);

void view_virtual_paths(const Embedding& _em, const pm::edge_attribute<VirtualPath>& _virtual_paths, float _width = default_line_width);

void view_edge(const pm::vertex_attribute<tg::pos3>& _pos, const pm::edge_handle& _e, const tg::color3& _color, float _width = default_line_width);
void view_vertex(const pm::vertex_attribute<tg::pos3>& _pos, const pm::vertex_handle& _v, const tg::color3& _color, float _size = default_point_size);
void view_vertex(const pm::vertex_attribute<tg::dpos3>& _pos, const pm::vertex_handle& _v, const tg::color3& _color, float _size = default_point_size);

void view_param(const VertexParam& _param);
void view_param(const std::vector<pm::face_handle>& _fs, const HalfedgeParam& _param);

void view_quad_mesh(const pm::vertex_attribute<tg::pos3>& _q_pos, const pm::face_attribute<pm::face_handle>& _q_matching_layout_face);

glow::SharedTexture2D read_texture(const fs::path &_file_path);

inline auto default_style()
{
    // Implemented inline so we can use 'auto' because the returned type is an implementation detail.
    return gv::config(gv::no_grid, gv::no_outline, gv::background_color(RWTH_WHITE), gv::ssao_power(0.5f));
}

inline auto render_shadows_only()
{
    // Implemented inline so we can use 'auto' because the returned type is an implementation detail.
    return gv::config(
        gv::background_color(tg::color3(0.0f,0.0f,0.0f)),
        [](gv::SceneConfig& cfg) {
            cfg.enableForwardRendering = false;
            cfg.enableShadows = true;
        }
    );
}

inline auto render_objects_only()
{
    // Implemented inline so we can use 'auto' because the returned type is an implementation detail.
    return gv::config(
        gv::background_color(tg::color3(0.0f,0.0f,0.0f)),
        [](gv::SceneConfig& cfg) {
            cfg.enableForwardRendering = true;
            cfg.enableShadows = false;
        }
    );
}

}
