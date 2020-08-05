#include "VirtualPathConflictSentinel.hh"

#include <LayoutEmbedding/Connectivity.hh>

namespace LayoutEmbedding {

VirtualPathConflictSentinel::VirtualPathConflictSentinel(const Embedding& _em) :
    em(_em),
    v_label(em.target_mesh()),
    e_label(em.target_mesh()),
    f_label(em.target_mesh())
{
}

void VirtualPathConflictSentinel::insert(const pm::vertex_handle& _v, const VirtualPathConflictSentinel::Label& _l)
{
    if (v_label[_v].is_valid()) {
        global_conflicts.insert(_l);
        global_conflicts.insert(v_label[_v]);
    }
    v_label[_v] = _l;
}

void VirtualPathConflictSentinel::insert(const pm::edge_handle& _e, const VirtualPathConflictSentinel::Label& _l)
{
    if (e_label[_e].is_valid()) {
        global_conflicts.insert(_l);
        global_conflicts.insert(e_label[_e]);
    }
    e_label[_e] = _l;
}

void VirtualPathConflictSentinel::insert(const pm::face_handle& _f, const VirtualPathConflictSentinel::Label& _l)
{
    if (f_label[_f].is_valid()) {
        global_conflicts.insert(_l);
        global_conflicts.insert(f_label[_f]);
    }
    f_label[_f] = _l;
}

void VirtualPathConflictSentinel::insert_virtual_vertex(const VirtualVertex& _vv, const VirtualPathConflictSentinel::Label& _l)
{
    if (is_real_vertex(_vv)) {
        insert(real_vertex(_vv), _l);
    }
    else {
        insert(real_edge(_vv), _l);
    }
}

void VirtualPathConflictSentinel::insert_segment(const VirtualVertex& _vv0, const VirtualVertex& _vv1, const VirtualPathConflictSentinel::Label& _l)
{
    if (is_real_vertex(_vv0)) {
        if (is_real_vertex(_vv1)) {
            // (V,V) case
            const auto& v0 = real_vertex(_vv0);
            const auto& v1 = real_vertex(_vv1);
            const auto& he = pm::halfedge_from_to(v0, v1);
            LE_ASSERT(he.is_valid());
            const auto& e = he.edge();
            insert(e, _l);
        }
        else {
            // (V,E) case
            const auto& v = real_vertex(_vv0);
            const auto& e = real_edge(_vv1);
            const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
    }
    else {
        if (is_real_vertex(_vv1)) {
            // (E,V) case
            const auto& e = real_edge(_vv0);
            const auto& v = real_vertex(_vv1);
            const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
        else {
            // (E,E) case
            const auto& e0 = real_edge(_vv0);
            const auto& e1 = real_edge(_vv1);
            const auto& f = common_face(e0, e1);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
    }
}

void VirtualPathConflictSentinel::insert_path(const VirtualPath& _path, const VirtualPathConflictSentinel::Label& _l)
{
    LE_ASSERT(_path.size() >= 2);
    LE_ASSERT(is_real_vertex(_path.front()));
    LE_ASSERT(is_real_vertex(_path.back()));

    // Note: We deliberately skip the first and last element
    for (int i = 1; i < _path.size() - 1; ++i) {
        insert_virtual_vertex(_path[i], _l);
    }

    // Path segments ("virtual edges")
    for (int i = 0; i < _path.size() - 1; ++i) {
        insert_segment(_path[i], _path[i+1], _l);
    }
}

}
