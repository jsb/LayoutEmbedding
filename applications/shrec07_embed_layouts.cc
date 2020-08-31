#include "shrec07.hh"

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/Praun2001.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>

using namespace LayoutEmbedding;

static bool screenshots_only = true;
static auto screenshot_size = tg::ivec2(1920, 1080);
static int screenshot_samples = 64;

namespace  {

void compute_embeddings(const std::string& _name, EmbeddingInput& _input)
{
    namespace fs = std::filesystem;

    const std::vector<std::string> algorithms = {
        "greedy",
        "bnb",
    };

    fs::create_directories(shrec_results_dir);
    const fs::path stats_path = shrec_results_dir / ("stats_" + _name + ".csv");
    {
        // CSV header
        std::ofstream f{stats_path};
        f << "layout_edges,algorithm,runtime,score" << std::endl;
    }

    for (const auto& algorithm : algorithms) {
        Embedding em(_input);
        const pm::Mesh& l_m = em.layout_mesh();
        const pm::Mesh& t_m = em.target_mesh();

        glow::timing::CpuTimer timer;

        if (algorithm == "bnb") {
            BranchAndBoundSettings settings;
            settings.use_hashing = true;
            settings.time_limit = 10 * 60;
            branch_and_bound(em, settings);
        }
        else if (algorithm == "greedy") {
            Praun2001Settings settings;
            settings.insertion_order = Praun2001Settings::InsertionOrder::BestFirst;
            settings.use_swirl_detection = false;
            praun2001(em, settings);
        }
        else {
            LE_ASSERT(false);
        }

        // Stats
        double optimization_time = timer.elapsedSeconds();
        double score = em.total_embedded_path_length();

        if (!em.is_complete()) {
            score = std::numeric_limits<double>::infinity();
        }

        std::cout << "Optimization took " << optimization_time << " s" << std::endl;
        std::cout << "Total embedding length: " << score << std::endl;
        {
            std::ofstream f{stats_path, std::ofstream::app};
            f << l_m.edges().size() << ",";
            f << algorithm << ",";
            f << optimization_time << ",";
            f << score << std::endl;
        }

        // Visualization
        auto cfg_style = gv::config(gv::no_grid, gv::no_outline, gv::background_color(RWTH_WHITE));

        // TODO: Camera position
        if (screenshots_only) {
            {
                const fs::path screenshot_path = shrec_results_dir / ("screenshot_" + _name + "_layout.png");
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string()));
                view_layout(em);
            }
            {
                const fs::path screenshot_path = shrec_results_dir / ("screenshot_" + _name + "_" + algorithm + "_target.png");
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string()));
                view_target(em);
            }
        }
        else {
            auto g = gv::grid();
            {
                view_layout(em);
            }
            {
                view_target(em);
            }
        }

        // Save to file
        fs::path saved_embeddings_dir = shrec_results_dir / "saved_embeddings";
        fs::create_directories(saved_embeddings_dir);
        fs::path embedding_path = saved_embeddings_dir / (_name + "_" + algorithm);
        em.save(embedding_path);
    }
}

// Warning:
// - Loses all attributes stored on edges, halfedges, and faces.
// - May change the indices of edges and halfedges.
void invert_mesh(pm::Mesh& _m)
{
    // Remember face connectivity
    std::vector<std::vector<pm::vertex_handle>> fv_indices;
    for (const auto f : _m.faces()) {
        std::vector<pm::vertex_handle> local_fv_indices;
        for (const auto v : f.vertices()) {
            local_fv_indices.push_back(v);
        }
        fv_indices.push_back(std::move(local_fv_indices));
    }

    // Remove all edges and faces
    for (const auto e : _m.edges()) {
        _m.edges().remove(e);
    }

    // Rebuild the mesh using inverted faces
    for (auto& local_fv_indices : fv_indices) {
        std::reverse(local_fv_indices.begin(), local_fv_indices.end());
        _m.faces().add(local_fv_indices);
    }

    _m.compactify();
}

}

int main()
{
    namespace fs = std::filesystem;

    glow::glfw::GlfwContext ctx;

    LE_ASSERT(fs::exists(shrec_dir));
    LE_ASSERT(fs::exists(shrec_corrs_dir));
    LE_ASSERT(fs::exists(shrec_meshes_dir));
    LE_ASSERT(fs::exists(shrec_layouts_dir));

    for (const int category : shrec_categories) {
        const fs::path layout_mesh_path = shrec_layouts_dir / (std::to_string(category) + ".obj");
        if (!fs::is_regular_file(layout_mesh_path)) {
            std::cout << "Could not find layout mesh " << layout_mesh_path << ". Skipping." << std::endl;
            continue;
        }

        for (int mesh_index = 0; mesh_index < shrec_meshes_per_category; ++mesh_index) {
            const int mesh_id = (category - 1) * shrec_meshes_per_category + mesh_index + 1;

            const fs::path target_mesh_path = shrec_meshes_dir / (std::to_string(mesh_id) + ".off");
            if (!fs::is_regular_file(target_mesh_path)) {
                std::cout << "Could not find target mesh " << target_mesh_path << ". Skipping." << std::endl;
                continue;
            }
            const fs::path corrs_path = shrec_corrs_dir / (std::to_string(mesh_id) + ".vts");
            if (!fs::is_regular_file(corrs_path)) {
                std::cout << "Could not find correspondence file " << corrs_path << ". Skipping." << std::endl;
                continue;
            }

            EmbeddingInput input;

            pm::load(layout_mesh_path, input.l_m, input.l_pos);
            std::cout << "Layout Mesh: ";
            std::cout << input.l_m.vertices().size() << " vertices, ";
            std::cout << input.l_m.edges().size() << " edges, ";
            std::cout << input.l_m.faces().size() << " faces. ";
            std::cout << "χ = " << pm::euler_characteristic(input.l_m) << std::endl;

            if (shrec_flipped_landmarks.count(mesh_id)) {
                std::cout << "This object is flipped. Inverting layout mesh." << std::endl;
                invert_mesh(input.l_m);
            }

            pm::load(target_mesh_path, input.t_m, input.t_pos);
            std::cout << "Target Mesh: ";
            std::cout << input.t_m.vertices().size() << " vertices, ";
            std::cout << input.t_m.edges().size() << " edges, ";
            std::cout << input.t_m.faces().size() << " faces. ";
            std::cout << "χ = " << pm::euler_characteristic(input.t_m) << std::endl;

            if (pm::euler_characteristic(input.l_m) != pm::euler_characteristic(input.t_m)) {
                std::cout << "Euler characteristic does not match. Skipping." << std::endl;
                continue;
            }

            // Load landmarks
            const auto landmark_ids = load_landmarks(corrs_path);
            std::cout << landmark_ids.size() << " landmarks." << std::endl;
            if (landmark_ids.size() != input.l_m.vertices().size()) {
                std::cout << "Wrong number of landmarks. Skipping." << std::endl;
                continue;
            }
            for (size_t i = 0; i < landmark_ids.size(); ++i) {
                const auto l_v = input.l_m.vertices()[i];
                const auto t_v = input.t_m.vertices()[landmark_ids[i]];
                input.l_matching_vertex[l_v] = t_v;
            }

            compute_embeddings(std::to_string(mesh_id), input);
        }
    }
}
