#include "VirtualPathConflictSentinel.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>

#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/HaltonColorGenerator.hh>
#include <glow-extras/viewer/view.hh>

namespace LayoutEmbedding {

VirtualPathConflictSentinel::VirtualPathConflictSentinel(const Embedding& _em) :
    em(_em),
    v_label(em.target_mesh()),
    e_label(em.target_mesh()),
    f_label(em.target_mesh()),
    l_port(em.layout_mesh())
{
}

void VirtualPathConflictSentinel::insert(const pm::vertex_handle& _v, const VirtualPathConflictSentinel::Label& _l)
{
    for (const auto& prev_l : v_label[_v]) {
        mark_conflicting(_l, prev_l);
    }
    v_label[_v].insert(_l);
}

void VirtualPathConflictSentinel::insert(const pm::edge_handle& _e, const VirtualPathConflictSentinel::Label& _l)
{
    for (const auto& prev_l : e_label[_e]) {
        mark_conflicting(_l, prev_l);
    }
    e_label[_e].insert(_l);
}

void VirtualPathConflictSentinel::insert(const pm::face_handle& _f, const VirtualPathConflictSentinel::Label& _l)
{
    for (const auto& prev_l : f_label[_f]) {
        mark_conflicting(_l, prev_l);
    }
    f_label[_f].insert(_l);
}

void VirtualPathConflictSentinel::insert_virtual_vertex(const VirtualVertex& _vv, const VirtualPathConflictSentinel::Label& _l)
{
    if (is_real_vertex(_vv)) {
        insert(real_vertex(_vv, em.target_mesh()), _l);
    }
    else {
        insert(real_edge(_vv, em.target_mesh()), _l);
    }
}

void VirtualPathConflictSentinel::insert_segment(const VirtualVertex& _vv0, const VirtualVertex& _vv1, const VirtualPathConflictSentinel::Label& _l)
{
    if (is_real_vertex(_vv0)) {
        if (is_real_vertex(_vv1)) {
            // (V,V) case
            const auto& v0 = real_vertex(_vv0, em.target_mesh());
            const auto& v1 = real_vertex(_vv1, em.target_mesh());

            const auto& he = pm::halfedge_from_to(v0, v1);
            LE_ASSERT(he.is_valid());
            const auto& e = he.edge();
            insert(e, _l);
        }
        else {
            // (V,E) case
            const auto& v = real_vertex(_vv0, em.target_mesh());
            const auto& e = real_edge(_vv1, em.target_mesh());

            const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
    }
    else {
        if (is_real_vertex(_vv1)) {
            // (E,V) case
            const auto& e = real_edge(_vv0, em.target_mesh());
            const auto& v = real_vertex(_vv1, em.target_mesh());

            const auto& f = triangle_with_edge_and_opposite_vertex(e, v);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
        else {
            // (E,E) case
            const auto& e0 = real_edge(_vv0, em.target_mesh());
            const auto& e1 = real_edge(_vv1, em.target_mesh());

            const auto& f = common_face(e0, e1);
            LE_ASSERT(f.is_valid());
            insert(f, _l);
        }
    }
}

void VirtualPathConflictSentinel::insert_path(const VirtualPath& _path, const VirtualPathConflictSentinel::Label& _l)
{
    LE_ASSERT(_path.size() >= 2);

    // Note: We deliberately skip the first and last element
    for (int i = 1; i < _path.size() - 1; ++i) {
        insert_virtual_vertex(_path[i], _l);
    }

    // Path segments ("virtual edges")
    for (int i = 0; i < _path.size() - 1; ++i) {
        insert_segment(_path[i], _path[i+1], _l);
    }

    // Additionally remember the directions (ports) through wich the path leaves / enters its endpoints.
    LE_ASSERT(is_real_vertex(_path.front()));
    LE_ASSERT(is_real_vertex(_path.back()));

    // Warning: Here we rely on the assumption that for each edge l_e, the corresponding path was traced
    // using find_shortest_path(l_e.halfedgeA());
    const pm::edge_handle l_e = em.layout_mesh().edges()[_l];
    LE_ASSERT(em.matching_target_vertex(l_e.halfedgeA().vertex_from()) == real_vertex(_path.front()));
    LE_ASSERT(em.matching_target_vertex(l_e.halfedgeA().vertex_to()) == real_vertex(_path.back()));

    const VirtualPort port_A(real_vertex(_path[0], em.target_mesh()), _path[1]);
    const VirtualPort port_B(real_vertex(_path[_path.size()-1], em.target_mesh()), _path[_path.size()-2]);
    // Links from layout to target
    l_port[l_e.halfedgeA()] = port_A;
    l_port[l_e.halfedgeB()] = port_B;
}

void VirtualPathConflictSentinel::mark_conflicting(const VirtualPathConflictSentinel::Label& _a, const VirtualPathConflictSentinel::Label& _b)
{
    if (_a == _b) {
        return;
    }

    global_conflicts.insert(_a);
    global_conflicts.insert(_b);

    const Conflict sorted = std::minmax(_a, _b);
    global_conflict_relation.insert(sorted);
}

void VirtualPathConflictSentinel::check_path_ordering()
{
    struct VirtualPortProperty
    {
        std::map<pm::vertex_index, LabelSet> v_labels;
        std::map<pm::edge_index, LabelSet> e_labels;

        LabelSet& operator[](const VirtualPort& _vp)
        {
            const VirtualVertex& vv = _vp.to;
            if (is_real_vertex(vv)) {
                return v_labels[real_vertex(vv)];
            }
            else {
                return e_labels[real_edge(vv)];
            }
        }
    };

    // DEBUG
    pm::edge_attribute<tg::color3> l_e_color(em.layout_mesh());
    HaltonColorGenerator hcg;
    for (const auto l_e : em.layout_mesh().edges()) {
        l_e_color[l_e] = hcg.generate_next_color();
    }

    for (const auto l_v : em.layout_mesh().vertices()) {
        // Create a local lookup table of the labels of incident candidate paths
        VirtualPortProperty labels;
        for (const auto l_he : l_v.outgoing_halfedges()) {
            if (!em.is_embedded(l_he)) {
                const VirtualPort& p = l_port[l_he];
                if (is_real_vertex(p.to)) {
                    const auto& t_v = em.matching_target_vertex(l_v);
                    LE_ASSERT(t_v.is_valid());
                    const auto& t_v_to = real_vertex(p.to, em.target_mesh());
                    LE_ASSERT(t_v_to.is_valid());
                    const auto& t_he = pm::halfedge_from_to(t_v, t_v_to);
                    LE_ASSERT(t_he.is_valid());
                    const auto& t_e = t_he.edge();
                    LE_ASSERT(t_e.is_valid());
                    LE_ASSERT(!em.is_blocked(t_e));
                }
                labels[p].insert(l_he.edge());
            }
        }

        const auto view_layout_ring = [&]() {
            auto v = gv::view();
            const int valence = pm::valence(l_v);
            int i = 0;
            for (const auto l_he : l_v.outgoing_halfedges()) {
                const auto& l_e = l_he.edge();
                const auto angle = (tg::tau<float> * i) / valence;
                const auto p0 = tg::pos3{0.0f, 0.0f, 0.0f};
                const auto p1 = tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};

                auto color = RWTH_WHITE;
                if (em.is_embedded(l_e)) {
                    color = RWTH_BLACK;
                }
                else {
                    color = l_e_color[l_e];
                }

                gv::view(gv::lines(tg::segment3{p0, p1}).line_width_px(4.0f), color, gv::no_shading);
                ++i;
            }
        };

        const auto view_layout_halfedge = [&](const pm::halfedge_handle& _l_he, const tg::color3& _color) {
            auto v = gv::view();
            const int valence = pm::valence(l_v);
            int i = 0;
            for (const auto l_he : l_v.outgoing_halfedges()) {
                if (l_he == _l_he) {
                    const auto angle = (tg::tau<float> * i) / valence;
                    const auto p0 = 1.1f * tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};
                    const auto p1 = 1.2f * tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};
                    gv::view(gv::lines(tg::segment3{p0, p1}).line_width_px(6.0f), _color, gv::no_shading);
                }
                ++i;
            }
        };

        const auto view_target_ring = [&]() {
            auto v = gv::view();
            const auto t_v = em.matching_target_vertex(l_v);
            const int valence = 2 * pm::valence(t_v); // virtual valence
            int i = 0;

            const auto start_port = VirtualPort(t_v.any_outgoing_halfedge());
            auto port = start_port;
            do {
                if (is_real_vertex(port.to)) {
                    const auto& t_he = pm::halfedge_from_to(t_v, real_vertex(port.to, em.target_mesh()));
                    const auto& t_e = t_he.edge();
                    const auto angle = (tg::tau<float> * i) / valence;
                    const auto p0 = tg::pos3{0.0f, 0.0f, 0.0f};
                    const auto p1 = tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};

                    auto color = RWTH_WHITE;
                    if (em.is_blocked(t_e)) {
                        color = RWTH_BLACK;
                    }

                    gv::view(gv::lines(tg::segment3{p0, p1}).line_width_px(4.0f), color, gv::no_shading);
                }

                port = port.rotated_ccw();
                ++i;
            }
            while (port != start_port);
        };

        const auto view_target_port = [&](const VirtualPort& _port, const tg::color3& _color) {
            auto v = gv::view();
            const auto t_v = em.matching_target_vertex(l_v);
            const int valence = 2 * pm::valence(t_v); // virtual valence
            int i = 0;
            const auto start_port = VirtualPort(t_v.any_outgoing_halfedge());
            auto port = start_port;
            do {
                if (port == _port) {
                    const auto angle = (tg::tau<float> * i) / valence;
                    const auto p0 = 1.1f * tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};
                    const auto p1 = 1.2f * tg::pos3{tg::cos(angle), tg::sin(angle), 0.0f};

                    gv::view(gv::lines(tg::segment3{p0, p1}).line_width_px(6.0f), _color, gv::no_shading);
                }

                port = port.rotated_ccw();
                ++i;
            }
            while (port != start_port);
        };

        const auto view_candidate_ports = [&]() {
            auto v = gv::view();
            const auto t_v = em.matching_target_vertex(l_v);
            const int valence = 2 * pm::valence(t_v); // virtual valence
            int i = 0;
            const auto start_port = VirtualPort(t_v.any_outgoing_halfedge());
            auto port = start_port;
            do {
                float z = 0.0f;
                for (const auto& l : labels[port]) {
                    z += 0.05f;
                    const auto angle = (tg::tau<float> * i) / valence;
                    const auto p0 = tg::pos3{0.0f, 0.0f, z};
                    const auto p1 = tg::pos3{tg::cos(angle), tg::sin(angle), z};
                    const auto color = l_e_color[l];
                    gv::view(gv::lines(tg::segment3{p0, p1}).line_width_px(4.0f), color, gv::no_shading);
                }
                port = port.rotated_ccw();
                ++i;
            }
            while (port != start_port);
        };

        const auto reachable_by_sweep_ccw_in_sector = [&](VirtualPort _start, const VirtualPort& _end) {
            LE_ASSERT(_start.from == _end.from);
            if (_start == _end) {
                // If both ports are identical then the corresponding paths are conflicting.
                return false;
            }
            while (_start != _end) {
                _start = _start.rotated_ccw();
                if (is_real_vertex(_start.to)) {
                    auto t_he = pm::halfedge_from_to(_start.from, real_vertex(_start.to, em.target_mesh()));
                    if (em.is_blocked(t_he.edge())) {
                        // Reached a sector boundary!
                        return false;
                    }
                }
            }
            return true;
        };

        const auto mark_and_sweep_cw_in_sector = [&](VirtualPort _start, const VirtualPort& _end, const Label& _l) {
            LE_ASSERT(_start.from == _end.from);
            const auto t_v = _start.from;

            if (is_real_vertex(_start.to)) {
                auto t_he = pm::halfedge_from_to(t_v, real_vertex(_start.to, em.target_mesh()));

                if (em.is_blocked(t_he.edge())) {
                    {
                        auto g = gv::grid();
                        {
                            auto v = gv::view();
                            view_layout_ring();
                            //view_layout_halfedge(l_v.any_outgoing_halfedge(), RWTH_MAGENTA);
                        }
                        {
                            auto v = gv::view();
                            view_target_ring();
                            view_candidate_ports();
                            //const auto t_v = em.matching_target_vertex(l_v);
                            //view_target_port(VirtualPort(t_v.any_outgoing_halfedge()), RWTH_MAGENTA);
                            view_target_port(_start, RWTH_BLUE);
                            view_target_port(_end, RWTH_RED);
                        }
                    }
                    LE_ERROR_THROW("");
                }
            }
            if (is_real_vertex(_end.to)) {
                auto t_he = pm::halfedge_from_to(t_v, real_vertex(_end.to, em.target_mesh()));
                LE_ASSERT(!em.is_blocked(t_he.edge()));
            }

            // Mark all encountered labels as conflicting.
            for (const auto& l : labels[_start]) {
                mark_conflicting(_l, l);
            }

            while (_start != _end) {
                _start = _start.rotated_cw();
                if (is_real_vertex(_start.to)) {
                    auto t_he = pm::halfedge_from_to(_start.from, real_vertex(_start.to, em.target_mesh()));
                    LE_ASSERT(!em.is_blocked(t_he.edge()));
                }
                // Mark all encountered labels as conflicting.
                for (const auto& l : labels[_start]) {
                    mark_conflicting(_l, l);
                }
            }
        };

        bool vertex_has_sectors = false;
        for (const auto l_sector_boundary_he : l_v.outgoing_halfedges()) {
            if (em.is_embedded(l_sector_boundary_he)) {
                vertex_has_sectors = true;

                // If a halfedge is embedded, the part following it is one "sector".
                // We now visit all layout halfedges in this sector (l_he_in_sector) in a CCW order
                // (i.e. until we reach another "embedded" layout halfedge).
                auto l_current_he_in_sector = rotated_ccw(l_sector_boundary_he);

                if (em.is_embedded(l_current_he_in_sector)) {
                    // Empty sector. Early out.
                    continue;
                }

                // Meanwhile, we keep track of the embedded directions (represented by current_port) of the corresponding layout halfedges in the sector.
                // We will check whether this direction also keeps "increasing" monotonically (a.k.a. rotating CCW) as we advance.
                // If current_port "decreases" (rotates CW), this introduces potential conflicts.
                // In that case, all ports that are swept over by a decreasing update of current_port will be marked as conflicting.
                VirtualPort current_port = l_port[l_current_he_in_sector];

                // Visit all layout halfedges in the current sector.
                while (!em.is_embedded(l_current_he_in_sector)) {
                    LE_ASSERT(current_port.is_valid());
                    LE_ASSERT(current_port.from == em.matching_target_vertex(l_v));

                    auto l_next_he_in_sector = rotated_ccw(l_current_he_in_sector);
                    if (em.is_embedded(l_next_he_in_sector)) {
                        break; // Reached end of sector
                    }
                    auto next_port = l_port[l_next_he_in_sector];
                    LE_ASSERT(next_port.is_valid());
                    LE_ASSERT(next_port.from == em.matching_target_vertex(l_v));

                    if (false) {
                        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
                        auto g = gv::grid();
                        {
                            auto v = gv::view();
                            view_layout_ring();
                            view_layout_halfedge(l_current_he_in_sector, RWTH_BLUE);
                            view_layout_halfedge(l_next_he_in_sector, RWTH_RED);
                        }
                        {
                            auto v = gv::view();
                            view_target_ring();
                            view_candidate_ports();
                            view_target_port(current_port, RWTH_BLUE);
                            view_target_port(next_port, RWTH_RED);
                        }
                    }

                    // Try to reach next_port from current_port using CCW rotations (without leaving the sector).
                    if (reachable_by_sweep_ccw_in_sector(current_port, next_port)) {
                        // All good, proceed.
                    }
                    else {
                        // Reach next_port from current_port using CW rotations (without leaving the sector).
                        // All candidate edges that are visited during the CW sweep will be marked "conflicting".
                        const Label& l = l_current_he_in_sector.edge().idx;
                        mark_and_sweep_cw_in_sector(current_port, next_port, l);
                    }

                    // Advance to the next halfedge in this sector.
                    l_current_he_in_sector = l_next_he_in_sector;
                    current_port = next_port;
                }
            }
        }

        if (!vertex_has_sectors) {
            // No sectors around the vertex (yet).
            // The best we can do is verify that the cyclic order of embedded edges matches that of the layout.
            // If so, there are no (additional) conflicts. Otherwise, we consider all edges around this vertex as conflicting.
            // TODO: Is it really necessary to mark all outgoing paths as conflicting in that case?
            // Can we do something that causes less branching?

            const auto l_he_start = l_v.any_outgoing_halfedge();
            auto l_he = l_he_start;
            LE_ASSERT(!em.is_embedded(l_he));

            const VirtualPort t_port_start = l_port[l_he_start];
            VirtualPort t_port = l_port[l_he];

            bool cyclic_conflict = false;
            do {
                LE_ASSERT(!em.is_embedded(l_he));
                const auto l_he_next = rotated_ccw(l_he);
                LE_ASSERT(!em.is_embedded(l_he_next));

                const auto t_port_next = l_port[l_he];

                // Try to reach t_port_next from t_port via CCW rotations without crossing t_port_start.
                while (t_port != t_port_next) {
                    t_port = t_port.rotated_ccw();

                    if (t_port == t_port_start) {
                        // We have cycled once around the embedded vertex before completing a cycle in the layout.
                        cyclic_conflict = true;
                        break;
                    }
                }

                l_he = l_he_next;
            }
            while (l_he != l_he_start && !cyclic_conflict);

            if (cyclic_conflict) {
                // Mark all surrounding edges as "conflicting"
                for (const auto l_e_A : l_v.edges()) {
                    for (const auto l_e_B : l_v.edges()) {
                        mark_conflicting(l_e_A, l_e_B);
                    }
                }
            }
        }
    }
}

}
