#include "VirtualPathConflictSentinel.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>

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
    LE_ASSERT(!em.is_embedded(_a));
    LE_ASSERT(!em.is_embedded(_b));

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
    for (const auto l_v : em.layout_mesh().vertices()) {
        bool vertex_has_sectors = false;
        for (const auto l_sector_boundary_he : l_v.outgoing_halfedges()) {
            if (em.is_embedded(l_sector_boundary_he)) {
                vertex_has_sectors = true;

                // A list of all unembedded layout edges that are in this sector,
                // along with their corresponding embedded ports
                std::vector<Label> labels_in_sector;
                std::vector<VirtualPort> embedded_ports_in_sector;
                {
                    auto l_he_in_sector = rotated_ccw(l_sector_boundary_he);
                    while (!em.is_embedded(l_he_in_sector)) {
                        labels_in_sector.push_back(l_he_in_sector.edge());
                        embedded_ports_in_sector.push_back(l_port[l_he_in_sector]);
                        l_he_in_sector = rotated_ccw(l_he_in_sector);
                    }
                }
                LE_ASSERT_EQ(labels_in_sector.size(), embedded_ports_in_sector.size());

                // This map assigns each unembedded layout edge in the sector an integer position
                // that corresponds to its index in the fan of possible outgoing ports in the
                // corresponding sector on the target mesh.
                std::map<Label, int> embedded_port_pos;
                {
                    const auto& t_v = em.matching_target_vertex(l_v);
                    const auto& t_he = em.get_embedded_target_halfedge(l_sector_boundary_he);
                    const auto start_port = VirtualPort(t_v, t_he.vertex_to());
                    auto current_port = start_port.rotated_ccw();
                    int current_port_pos = 0;
                    while (true) {
                        if (is_real_vertex(current_port.to)) {
                            const auto& t_he_current = current_port.real_halfedge();
                            if (em.is_blocked(t_he_current.edge())) {
                                // Reached end of sector
                                break;
                            }
                        }

                        // Store positions
                        LE_ASSERT(labels_in_sector.size() == embedded_ports_in_sector.size());
                        for (std::size_t i = 0; i < labels_in_sector.size(); ++i) {
                            const auto& label = labels_in_sector[i];
                            const auto& embedded_port = embedded_ports_in_sector[i];
                            if (embedded_port == current_port) {
                                embedded_port_pos[label] = current_port_pos;
                            }
                        }

                        current_port = current_port.rotated_ccw();
                        ++current_port_pos;
                    }
                }
                // All incident edges in the layout sector should have a corresponding port in the target sector!
                LE_ASSERT(labels_in_sector.size() == embedded_port_pos.size());

                // Detect conflicting edges
                for (std::size_t i = 0; i < labels_in_sector.size(); ++i) {
                    const auto& label = labels_in_sector[i];
                    const auto& port_pos = embedded_port_pos.at(label);

                    for (std::size_t i_left = 0; i_left < i; ++i_left) {
                        const auto& label_left = labels_in_sector[i_left];
                        const auto& port_pos_left = embedded_port_pos.at(label_left);
                        if (port_pos_left >= port_pos) {
                            mark_conflicting(label, label_left);
                        }
                    }

                    for (std::size_t i_right = i + 1; i_right < labels_in_sector.size(); ++i_right) {
                        const auto& label_right = labels_in_sector[i_right];
                        const auto& port_pos_right = embedded_port_pos.at(label_right);
                        if (port_pos_right <= port_pos) {
                            mark_conflicting(label, label_right);
                        }
                    }
                }
            }
        }

        if (!vertex_has_sectors) {
            // Save back references -- from VirtualPorts around this vertex to corresponding layout halfedges.
            std::map<VirtualPort, std::set<Label>> labels_at_port;
            for (const auto l_he : l_v.outgoing_halfedges()) {
                LE_ASSERT(!em.is_embedded(l_he));
                const auto& port = l_port[l_he];
                const auto& label = l_he.edge();
                labels_at_port[port].insert(label);
            }

            // Detect local violations of cyclic order
            for (const pm::halfedge_handle l_he : l_v.outgoing_halfedges()) {
                const pm::halfedge_handle& l_he_prev = rotated_cw(l_he);
                const pm::halfedge_handle& l_he_next = rotated_ccw(l_he);

                const Label& l_prev = l_he_prev.edge();
                const Label& l      = l_he.edge();
                const Label& l_next = l_he_next.edge();

                const VirtualPort& port_prev = l_port[l_he_prev];
                const VirtualPort& port      = l_port[l_he];
                const VirtualPort& port_next = l_port[l_he_next];

                LE_ASSERT(port_prev.is_valid());
                LE_ASSERT(port.is_valid());
                LE_ASSERT(port_next.is_valid());

                // We check that port lies between port_prev and port_next.
                // To do this, we start at port_prev, and rotate CCW until port is found.
                // If any other embedded edge is encountered first, we have detected a conflict.
                // We then repeat the same thing starting from port, trying to reach port_next.

                bool valid = true;
                auto port_current = port_prev;

                // Check the sector from port_prev to port
                while (port_current != port) {
                    for (const auto& l_at_port : labels_at_port[port_current]) {
                        // Any other labels at port_current?
                        if (l_at_port != l_prev) {
                            // --> Conflict
                            valid = false;
                            break;
                        }
                    }
                    port_current = port_current.rotated_ccw();
                }

                LE_ASSERT(port_current == port);

                // Check the sector from port to port_next
                while (port_current != port_next) {
                    for (const auto& l_at_port : labels_at_port[port_current]) {
                        // Any other labels at port_current?
                        if (l_at_port != l) {
                            // --> Conflict
                            valid = false;
                            break;
                        }
                    }
                    port_current = port_current.rotated_ccw();
                }

                LE_ASSERT(port_current == port_next);

                for (const auto& l_at_port : labels_at_port[port_next]) {
                    // Any other labels at port_next?
                    if (l_at_port != l_next) {
                        // --> Conflict
                        valid = false;
                        break;
                    }
                }

                if (!valid) {
                    // The current label (l) is marked as conflicting with all other incident labels around the vertex
                    for (const auto other_l : l_v.edges()) {
                        mark_conflicting(l, other_l);
                    }
                }
            }
        }
    }
}

}
