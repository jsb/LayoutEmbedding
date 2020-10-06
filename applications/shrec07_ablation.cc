/**
  * Runs an ablation study on a subset of the SHREC07 dataset.
  *
  * Instructions:
  *
  *     * Run shrec07_generate_layouts before running this file (~24h).
  *     * Running this experiment might take a while (~24h).
  *
  * Output files can be found in <build-folder>/output/bnb_ablation.
  *
  */

#include "shrec07.hh"

#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/Util/Assert.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <polymesh/algorithms/triangulate.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;

    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    LE_ASSERT(fs::exists(shrec_dir));
    LE_ASSERT(fs::exists(shrec_corrs_dir));
    LE_ASSERT(fs::exists(shrec_meshes_dir));
    LE_ASSERT(fs::exists(shrec_layouts_dir));

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path bnb_ablation_output_dir = output_dir / "bnb_ablation";
    const fs::path stats_path = bnb_ablation_output_dir / "stats_shrec07.csv";

    fs::create_directories(bnb_ablation_output_dir);
    {
        std::ofstream f(stats_path);
        f << "mesh_id,state_hashing,proactive_pruning,balanced_priority,advanced_lower_bounds,runtime,last_upper_bound_event_t,score" << std::endl;
    }

    struct Configuration
    {
        bool state_hashing = true;
        bool proactive_pruning = true;
        bool balanced_priority = true;
        bool advanced_lower_bounds = true;
    };
    std::vector<Configuration> configs;
    configs.push_back({true,  true,  true,  true });
    configs.push_back({false, true,  true,  true });
    configs.push_back({true,  false, true,  true });
    configs.push_back({true,  true,  false, true });
    configs.push_back({true,  true,  true,  false});

    const std::set<int> mesh_ids = { 1, 3, 4, 10, 11, 13, 44, 45, 46, 53, 54, 57, 58, 122, 123, 131, 132, 134, 135, 136, 137, 138, 139, 181, 182, 183, 184, 185, 186, 188, 190, 191, 192, 193, 194, 195, 197, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 223, 224, 225, 226, 227, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 304, 305, 306, 307, 308, 310, 312, 319, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 354, 388, 391, 399, 400 };

    for (const int category : shrec_categories) {
        const fs::path layout_mesh_path = shrec_layouts_dir / (std::to_string(category) + ".obj");
        if (!fs::is_regular_file(layout_mesh_path)) {
            std::cout << "Could not find layout mesh " << layout_mesh_path << ". Skipping." << std::endl;
            continue;
        }

        for (int mesh_index = 0; mesh_index < shrec_meshes_per_category; ++mesh_index) {
            const int mesh_id = (category - 1) * shrec_meshes_per_category + mesh_index + 1;

            if (mesh_ids.count(mesh_id) == 0) {
                continue;
            }

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
            if (!input.load(layout_mesh_path, target_mesh_path, corrs_path, LandmarkFormat::id_x_y_z)) {
                continue;
            }

            if (shrec_flipped_landmarks.count(mesh_id)) {
                std::cout << "This object is flipped. Inverting layout mesh." << std::endl;
                input.invert_layout();
            }

            input.normalize_surface_area();
            input.center_translation();

            for (const auto& config : configs) {
                // Generate random matching vertices
                Embedding em(input);

                BranchAndBoundSettings settings;
                settings.time_limit = 5 * 60;
                settings.use_greedy_init = false;
                settings.use_state_hashing = config.state_hashing;
                settings.use_proactive_pruning = config.proactive_pruning;
                settings.use_candidate_paths_for_lower_bounds = config.advanced_lower_bounds;
                if (config.balanced_priority) {
                    settings.priority = BranchAndBoundSettings::Priority::LowerBoundNonConflicting;
                }
                else {
                    settings.priority = BranchAndBoundSettings::Priority::LowerBound;
                }

                glow::timing::CpuTimer timer;
                auto result = branch_and_bound(em, settings);

                const double runtime = timer.elapsedSeconds();
                const double embedding_cost = em.is_complete() ? em.total_embedded_path_length() : std::numeric_limits<double>::infinity();

                double last_upper_bound_event_t = std::numeric_limits<double>::infinity();
                if (!result.upper_bound_events.empty()) {
                    last_upper_bound_event_t = result.upper_bound_events.back().t;
                }

                {
                    std::ofstream f{stats_path, std::ofstream::app};
                    f << mesh_id << ",";
                    f << config.state_hashing << ",";
                    f << config.proactive_pruning << ",";
                    f << config.balanced_priority << ",";
                    f << config.advanced_lower_bounds << ",";
                    f << runtime << ",";
                    f << last_upper_bound_event_t << ",";
                    f << embedding_cost << std::endl;
                }

                std::cout << "Mesh ID:               " << mesh_id << std::endl;
                std::cout << "State Hashing:         " << config.state_hashing << std::endl;
                std::cout << "Proactive Pruning:     " << config.proactive_pruning << std::endl;
                std::cout << "Balanced Priority:     " << config.balanced_priority << std::endl;
                std::cout << "Advanced Lower Bounds: " << config.advanced_lower_bounds << std::endl;
                std::cout << "Runtime:               " << runtime << std::endl;
                std::cout << "Last UB Event:         " << last_upper_bound_event_t << std::endl;
                std::cout << "Cost:                  " << embedding_cost << std::endl;
            }
        }
    }
}
