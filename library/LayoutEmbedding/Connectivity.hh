#pragma once

#include <polymesh/pm.hh>

namespace LayoutEmbedding {

bool adjacent(const pm::vertex_handle& _v0, const pm::vertex_handle& _v1);

bool incident(const pm::vertex_handle& _v, const pm::edge_handle& _e);
bool incident(const pm::edge_handle& _e, const pm::vertex_handle& _v);

/// The opposite vertex handle in a triangle (if it exists)
pm::vertex_handle opposite_vertex(const pm::halfedge_handle& _he);

/// The common vertex of two edges (if it exists)
pm::vertex_handle common_vertex(const pm::edge_handle& _e0, const pm::edge_handle& _e1);

/// The common face of two edges (if it exists)
pm::face_handle common_face(const pm::edge_handle& _e0, const pm::edge_handle& _e1);

/// Finds the triangle containing _e and _v such that _v is opposite of _e.
pm::face_handle triangle_with_edge_and_opposite_vertex(const pm::edge_handle& _e, const pm::vertex_handle& _v);

/// Rotates a halfedge clockwise around its vertex_from.
pm::halfedge_handle rotated_cw(const pm::halfedge_handle& _he);

/// Rotates a halfedge counterclockwise around its vertex_from.
pm::halfedge_handle rotated_ccw(const pm::halfedge_handle& _he);

}
