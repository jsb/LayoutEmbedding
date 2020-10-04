#include "Connectivity.hh"

#include <LayoutEmbedding/Assert.hh>

namespace LayoutEmbedding {

bool adjacent(const pm::vertex_handle& _v0, const pm::vertex_handle& _v1)
{
    LE_ASSERT(_v0.mesh == _v1.mesh);
    const auto he = pm::halfedge_from_to(_v0, _v1);
    return he.is_valid();
}

bool incident(const pm::vertex_handle& _v, const pm::edge_handle& _e)
{
    LE_ASSERT(_v.mesh == _e.mesh);
    return (_e.vertexA() == _v) || (_e.vertexB() == _v);
}

bool incident(const pm::edge_handle& _e, const pm::vertex_handle& _v)
{
    return incident(_v, _e);
}

pm::vertex_handle opposite_vertex(const pm::halfedge_handle& _he)
{
    if (_he.is_boundary()) {
        return pm::vertex_handle::invalid;
    }
    else {
        return _he.next().vertex_to();
    }
}

pm::vertex_handle common_vertex(const pm::edge_handle& _e0, const pm::edge_handle& _e1)
{
    LE_ASSERT(_e0 != _e1);
    if (_e0.vertexA() == _e1.vertexA()) {
        return _e0.vertexA();
    }
    else if (_e0.vertexA() == _e1.vertexB()) {
        return _e0.vertexA();
    }
    else if (_e0.vertexB() == _e1.vertexA()) {
        return _e0.vertexB();
    }
    else if (_e0.vertexB() == _e1.vertexB()) {
        return _e0.vertexB();
    }
    else {
        return pm::vertex_handle::invalid;
    }
}

pm::face_handle common_face(const pm::edge_handle& _e0, const pm::edge_handle& _e1)
{
    LE_ASSERT(_e0 != _e1);
    if (_e0.faceA() == _e1.faceA()) {
        return _e0.faceA();
    }
    else if (_e0.faceA() == _e1.faceB()) {
        return _e0.faceA();
    }
    else if (_e0.faceB() == _e1.faceA()) {
        return _e0.faceB();
    }
    else if (_e0.faceB() == _e1.faceB()) {
        return _e0.faceB();
    }
    else {
        return pm::face_handle::invalid;
    }
}

pm::face_handle triangle_with_edge_and_opposite_vertex(const pm::edge_handle& _e, const pm::vertex_handle& _v)
{
    for (const auto& he : {_e.halfedgeA(), _e.halfedgeB()}) {
        if (opposite_vertex(he) == _v) {
            return he.face();
        }
    }
    return pm::face_handle::invalid;
}

pm::halfedge_handle rotated_cw(const pm::halfedge_handle& _he)
{
    return _he.opposite().next();
}

pm::halfedge_handle rotated_ccw(const pm::halfedge_handle& _he)
{
    return _he.prev().opposite();
}

}
