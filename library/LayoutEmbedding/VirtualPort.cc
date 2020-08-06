#include "VirtualPort.hh"

#include <LayoutEmbedding/Assert.hh>

namespace LayoutEmbedding {

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
    if (is_real_vertex(to)) {
        const auto he = pm::halfedge_from_to(from, real_vertex(to));
        const auto e_new = he.opposite().prev().edge();
        return {from, e_new};
    }
    else if (is_real_edge(to)) {
        const auto e = real_edge(to);
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
    LE_ASSERT(false);
    return {};
}

VirtualPort VirtualPort::rotated_ccw() const
{
    if (is_real_vertex(to)) {
        const auto he = pm::halfedge_from_to(from, real_vertex(to));
        const auto e_new = he.next().edge();
        return {from, e_new};
    }
    else if (is_real_edge(to)) {
        const auto e = real_edge(to);
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
    LE_ASSERT(false);
    return {};
}

}
