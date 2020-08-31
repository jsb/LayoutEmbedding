#include "shrec07.hh"

#include <polymesh/formats.hh>
#include <polymesh/algorithms/decimate.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>

using namespace LayoutEmbedding;

namespace  {

struct DecimateConfig : public pm::decimate_config<tg::pos3, tg::quadric3>
{
    using parent_t = typename pm::decimate_config<tg::pos3, tg::quadric3>;

    DecimateConfig(const pm::vertex_attribute<int>& _landmark_ids) :
        landmark_ids(_landmark_ids)
    {
    }

    bool is_collapse_allowed(pm::halfedge_handle h) const
    {
        return landmark_ids[h.vertex_from()] < 0;
    }

    const pm::vertex_attribute<int>& landmark_ids;
};

}

int main()
{
    namespace fs = std::filesystem;

    LE_ASSERT(fs::exists(shrec_dir));
    LE_ASSERT(fs::exists(shrec_corrs_dir));
    LE_ASSERT(fs::exists(shrec_meshes_dir));

    for (const int category : shrec_categories) {
        bool success = false;
        for (int mesh_index = 0; mesh_index < shrec_meshes_per_category; ++mesh_index) {
            const int mesh_id = (category - 1) * shrec_meshes_per_category + mesh_index + 1;
            std::cout << "Category " << category << ", mesh " << mesh_index << ": Mesh ID " << mesh_id << "." << std::endl;

            const fs::path mesh_path = shrec_meshes_dir / (std::to_string(mesh_id) + ".off");
            if (!fs::is_regular_file(mesh_path)) {
                std::cout << "Could not find mesh " << mesh_path << std::endl;
                continue;
            }
            const fs::path corrs_path = shrec_corrs_dir / (std::to_string(mesh_id) + ".vts");
            if (!fs::is_regular_file(corrs_path)) {
                std::cout << "Could not find landmark file " << corrs_path << std::endl;
                continue;
            }

            // Load mesh
            EmbeddingInput input;

            pm::load(mesh_path, input.t_m, input.t_pos);
            std::cout << "Target Mesh: ";
            std::cout << input.t_m.vertices().size() << " vertices, ";
            std::cout << input.t_m.edges().size() << " edges, ";
            std::cout << input.t_m.faces().size() << " faces. ";
            std::cout << "χ = " << pm::euler_characteristic(input.t_m) << std::endl;

            if (pm::euler_characteristic(input.t_m) != 2) {
                std::cout << "Mesh is not genus 0" << std::endl;
                continue;
            }

            // Load landmarks
            const auto landmarks = load_landmarks(corrs_path);
            std::cout << landmarks.size() << " landmarks." << std::endl;

            // Create Layout mesh
            input.l_m.copy_from(input.t_m);
            input.l_pos.copy_from(input.t_pos);

            // Constrained decimation
            pm::vertex_attribute<tg::quadric3> l_error(input.l_m);
            for (const auto l_v : input.l_m.vertices()) {
                for (const auto l_f : l_v.faces()) {
                    const auto& p = input.l_pos[l_v];
                    const auto& n = pm::face_normal(l_f, input.l_pos);
                    l_error[l_v].add_plane(p, n, 0.0);
                }
            }

            auto l_m_landmark_id = input.l_m.vertices().make_attribute<int>(-1);
            for (std::size_t i = 0; i < landmarks.size(); ++i) {
                const auto& l_v = input.l_m.vertices()[landmarks[i]];
                l_m_landmark_id[l_v] = i;
            }

            DecimateConfig decimate_config(l_m_landmark_id);
            pm::decimate(input.l_m, input.l_pos, l_error, decimate_config);
            input.l_m.compactify();

            std::cout << "Layout Mesh: ";
            std::cout << input.l_m.vertices().size() << " vertices, ";
            std::cout << input.l_m.edges().size() << " edges, ";
            std::cout << input.l_m.faces().size() << " faces. ";
            std::cout << "χ = " << pm::euler_characteristic(input.l_m) << std::endl;

            if (pm::euler_characteristic(input.l_m) != 2) {
                std::cout << "Decimated mesh is not genus 0" << std::endl;
                continue;
            }

            if(input.l_m.vertices().size() == landmarks.size()) {
                // Decimation successful
                success = true;
                std::cout << "Success!" << std::endl;

                // Bring layout vertices in the correct order
                auto p = std::vector<int>(input.l_m.vertices().size());
                for (const auto& l_v : input.l_m.vertices()) {
                    const int curr_idx = l_v.idx.value;
                    const int new_idx = l_m_landmark_id[l_v];
                    LE_ASSERT(new_idx >= 0);
                    p[curr_idx] = new_idx;
                }
                input.l_m.vertices().permute(p);

                fs::create_directories(shrec_layouts_dir);
                fs::path output_filename = shrec_layouts_dir / (std::to_string(category) + ".obj");
                pm::save(output_filename, input.l_pos);

                break;
            }
        }
        if (!success) {
            std::cout << "Warning: Could not generate a layout mesh by decimation" << std::endl;
        }
    }
}
