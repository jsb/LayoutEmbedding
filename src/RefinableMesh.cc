#include "RefinableMesh.hh"

#include "Connectivity.hh"

#include <iostream>

RefinableMesh make_refinable_mesh(pm::Mesh& _m, pm::vertex_attribute<tg::pos3>& _pos)
{
    RefinableMesh result{&_m, _m.copy(), &_pos, _m, _m, _m};
    for (const auto v : result.m->vertices()) {
        result.vertex_ancestor[v] = result.m_orig->vertices()[v.idx];
    }
    for (const auto e : result.m->edges()) {
        result.edge_ancestor[e] = result.m_orig->edges()[e.idx];
    }
    for (const auto f : result.m->faces()) {
        result.face_ancestor[f] = result.m_orig->faces()[f.idx];
    }
    return result;
}

pm::vertex_handle split_edge(RefinableMesh& _rm, const pm::edge_handle& _e)
{
    auto& m = *_rm.m;
    auto& pos = *_rm.pos;

    // Elements before the split
    const auto& old_e       = _e;
    const auto& old_he      = old_e.halfedgeA();
    const auto& old_v_from  = old_he.vertex_from();
    const auto& old_v_to    = old_he.vertex_to();
    const auto& old_v_left  = old_he.next().vertex_to();
    const auto& old_v_right = old_he.opposite().next().vertex_to();
    const auto& old_f_left  = old_he.face();
    const auto& old_f_right = old_he.opposite().face();

    // Remember old property values invalidated by the split
    const auto old_e_ancestor       = _rm.edge_ancestor[old_e];
    const auto old_f_left_ancestor  = _rm.face_ancestor[old_f_left];
    const auto old_f_right_ancestor = _rm.face_ancestor[old_f_right];

    // Split
    const auto& new_v = m.edges().split_and_triangulate(_e);
    pos[new_v] = tg::mix(pos[old_v_from], pos[old_v_to], 0.5);

    // Elements after the split
    const auto& new_he_from  = pm::halfedge_from_to(old_v_from, new_v);
    const auto& new_he_to    = pm::halfedge_from_to(new_v, old_v_to);
    const auto& new_he_left  = pm::halfedge_from_to(new_v, old_v_left);
    const auto& new_he_right = pm::halfedge_from_to(new_v, old_v_right);

    const auto& new_e_from  = new_he_from.edge();
    const auto& new_e_to    = new_he_to.edge();
    const auto& new_e_left  = new_he_left.edge();
    const auto& new_e_right = new_he_right.edge();

    const auto& new_f_from_left  = new_he_from.face();
    const auto& new_f_from_right = new_he_from.opposite().face();
    const auto& new_f_to_left    = new_he_to.face();
    const auto& new_f_to_right   = new_he_to.opposite().face();

    // Update element ancestors
    _rm.vertex_ancestor[new_v] = old_e_ancestor;

    _rm.edge_ancestor[new_e_from] = old_e_ancestor;
    _rm.edge_ancestor[new_e_to] = old_e_ancestor;
    _rm.edge_ancestor[new_e_left] = old_f_left_ancestor;
    _rm.edge_ancestor[new_e_right] = old_f_right_ancestor;

    _rm.face_ancestor[new_f_from_left] = old_f_left_ancestor;
    _rm.face_ancestor[new_f_from_right] = old_f_right_ancestor;
    _rm.face_ancestor[new_f_to_left] = old_f_left_ancestor;
    _rm.face_ancestor[new_f_to_right] = old_f_right_ancestor;

    return new_v;
}

bool is_on_original_mesh(const RefinableMesh& _rm, const pm::vertex_handle& _v)
{
    const auto& anc = _rm.vertex_ancestor[_v];
    return std::holds_alternative<pm::edge_handle>(anc) || std::holds_alternative<pm::vertex_handle>(anc);
}

bool is_on_original_mesh(const RefinableMesh& _rm, const pm::edge_handle& _e)
{
    const auto& anc = _rm.edge_ancestor[_e];
    return std::holds_alternative<pm::edge_handle>(anc);
}

bool is_original_vertex(const RefinableMesh& _rm, const pm::vertex_handle& _v)
{
    const auto& anc = _rm.vertex_ancestor[_v];
    return std::holds_alternative<pm::vertex_handle>(anc);
}

bool on_common_ancestor_edge(const RefinableMesh& _rm, const pm::vertex_handle& _v0, const pm::vertex_handle& _v1)
{
    const auto& anc0 = _rm.vertex_ancestor[_v0];
    const auto& anc1 = _rm.vertex_ancestor[_v1];
    if (auto anc0_e = std::get_if<pm::edge_handle>(&anc0)) {
        if (auto anc1_e = std::get_if<pm::edge_handle>(&anc1)) {
            // Edge-edge case
            if (*anc0_e == *anc1_e) {
                return true;
            }
        }
        else if (auto anc1_v = std::get_if<pm::vertex_handle>(&anc1)) {
            // Edge-vertex case
            if (incident(*anc0_e, *anc1_v)) {
                return true;
            }
        }
    }
    else if (auto anc0_v = std::get_if<pm::vertex_handle>(&anc0)) {
        if (auto anc1_e = std::get_if<pm::edge_handle>(&anc1)) {
            // Vertex-edge case
            if (incident(*anc1_e, *anc0_v)) {
                return true;
            }
        }
        else if (auto anc1_v = std::get_if<pm::vertex_handle>(&anc1)) {
            // Vertex-vertex case
            if (adjacent(*anc0_v, *anc1_v)) {
                return true;
            }
        }
    }
    return false;
}

void cleanup(RefinableMesh& _rm)
{
    auto& m = *_rm.m;

    int passes = 0;
    int collapses = 0;
    bool retry;
    do {
        retry = false;
        for (const auto he : m.halfedges()) {
            const auto& e = he.edge();
            const auto& v_from = he.vertex_from();
            const auto& v_to = he.vertex_to();

            // TODO: check for blocked elements

            // Original vertices of the input mesh must not be decimated
            if (is_original_vertex(_rm, v_from)) {
                continue;
            }

            const auto& he_left = he.prev().opposite();
            const auto& he_right = he.opposite().next();
            if (is_on_original_mesh(_rm, he_left.edge())) {
                continue;
            }
            if (is_on_original_mesh(_rm, he_right.edge())) {
                continue;
            }

            if (is_on_original_mesh(_rm, v_from)) {
                // v_from is a vertex on an original edge
                // It must only be collapsed along original edges
                if (!is_on_original_mesh(_rm, e)) {
                    continue;
                }
            }

            bool collapsible = true;
            for (const auto he_out : v_from.outgoing_halfedges()) {
                if ((he_out == he) || (he_out == he_left) || (he_out == he_right)) {
                    continue;
                }
                const auto& e_out = he_out.edge();
                const auto& anc_e = _rm.edge_ancestor[e];
                const auto& anc_e_out = _rm.edge_ancestor[e_out];
                if (std::holds_alternative<pm::edge_handle>(anc_e) && std::holds_alternative<pm::edge_handle>(anc_e_out)) {
                    if (anc_e == anc_e_out) {
                        continue;
                    }
                }
                if (on_common_ancestor_edge(_rm, he_out.vertex_to(), v_to)) {
                    collapsible = false;
                    break;
                }
            }
            if (!collapsible) {
                continue;
            }

            // Low-level consistency checks
            if (!pm::can_collapse(he)) {
                continue;
            }

            // Perform the collapse
            m.halfedges().collapse(he);
            ++collapses;
            retry = true;

            // TODO: handle updates of ancestors?
        }
        ++passes;
    }
    while (retry);

    std::cout << passes << " passes, " << collapses << " collapses." << std::endl;
}
