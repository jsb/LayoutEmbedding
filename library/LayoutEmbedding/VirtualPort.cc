#include "VirtualPort.hh"

#include <LayoutEmbedding/Assert.hh>

namespace LayoutEmbedding {

VirtualPort::VirtualPort(const polymesh::vertex_handle& _from, const VirtualVertex& _to) :
    from(_from),
    to(_to)
{
}

VirtualPort::VirtualPort(const polymesh::halfedge_handle& _real_he) :
    from(_real_he.vertex_from()),
    to(_real_he.vertex_to())
{
}

bool VirtualPort::operator==(const VirtualPort& _rhs) const
{
    return (from == _rhs.from) && (to == _rhs.to);
}

bool VirtualPort::operator!=(const VirtualPort& _rhs) const
{
    return !(*this == _rhs);
}

VirtualPort VirtualPort::rotated_cw() const
{
    const auto& m = *from.mesh;
    if (is_real_vertex(to)) {
        const auto he = pm::halfedge_from_to(from, real_vertex(to, m));
        LE_ASSERT(he.is_valid());
        const auto e_new = he.opposite().prev().edge();
        return {from, e_new};
    }
    else { // if (is_real_edge(to)) {
        const auto e = real_edge(to, m);
        LE_ASSERT(e.is_valid());
        auto he = pm::halfedge_handle::invalid;
        if (e.halfedgeA().next().vertex_to() == from) {
            he = e.halfedgeA();
        }
        else if (e.halfedgeB().next().vertex_to() == from) {
            he = e.halfedgeB();
        }
        LE_ASSERT(he.is_valid());
        const auto v_new = he.vertex_from();
        return {from, v_new};
    }
}

VirtualPort VirtualPort::rotated_ccw() const
{
    const auto& m = *from.mesh;
    if (is_real_vertex(to)) {
        const auto he = pm::halfedge_from_to(from, real_vertex(to, m));
        LE_ASSERT(he.is_valid());
        const auto e_new = he.next().edge();
        return {from, e_new};
    }
    else { // if (is_real_edge(to)) {
        const auto e = real_edge(to, m);
        LE_ASSERT(e.is_valid());
        auto he = pm::halfedge_handle::invalid;
        if (e.halfedgeA().next().vertex_to() == from) {
            he = e.halfedgeA();
        }
        else if (e.halfedgeB().next().vertex_to() == from) {
            he = e.halfedgeB();
        }
        LE_ASSERT(he.is_valid());
        const auto v_new = he.vertex_to();
        return {from, v_new};
    }
}

bool VirtualPort::is_valid() const
{
    return from.is_valid() && LayoutEmbedding::is_valid(to);
}

}
