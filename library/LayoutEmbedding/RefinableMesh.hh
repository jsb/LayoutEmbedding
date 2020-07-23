#pragma once

#include <polymesh/pm.hh>

#include <typed-geometry/tg.hh>

#include <variant>

using Ancestor = std::variant<
    pm::vertex_handle,
    pm::edge_handle,
    pm::face_handle
>;

struct RefinableMesh
{
    pm::Mesh* m;
    pm::unique_ptr<pm::Mesh> m_orig;
    pm::vertex_attribute<tg::pos3>* pos;
    pm::vertex_attribute<Ancestor> vertex_ancestor;
    pm::edge_attribute<Ancestor> edge_ancestor;
    pm::face_attribute<Ancestor> face_ancestor;
};

RefinableMesh make_refinable_mesh(pm::Mesh& _m, pm::vertex_attribute<tg::pos3>& _pos);

pm::vertex_handle split_edge(RefinableMesh& _rm, const pm::edge_handle& _e);

bool is_on_original_mesh(const RefinableMesh& _rm, const pm::vertex_handle& _v);
bool is_on_original_mesh(const RefinableMesh& _rm, const pm::edge_handle& _e);
bool is_original_vertex(const RefinableMesh& _rm, const pm::vertex_handle& _v);
bool on_common_ancestor_edge(const RefinableMesh& _rm, const pm::vertex_handle& _v0, const pm::vertex_handle& _v1);

void cleanup(RefinableMesh& _rm);
