#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;

    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path bnb_ablation_output_dir = output_dir / "bnb_ablation_cow_even_simpler";

    EmbeddingInput input;
    load(data_path / "models/target-meshes/cow_6938.obj", input.t_m, input.t_pos);
    load(data_path / "models/layouts/cow_simple.obj", input.l_m, input.l_pos);
    find_matching_vertices_by_proximity(input);
    input.normalize_surface_area();
    input.center_translation();

    fs::create_directories(bnb_ablation_output_dir);
    const fs::path stats_path = bnb_ablation_output_dir / "stats.csv";
    {
        std::ofstream f(stats_path);
        f << "seed,state_hashing,proactive_pruning,balanced_priority,advanced_lower_bounds,runtime,last_upper_bound_event_t,score" << std::endl;
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
    //configs.push_back({true,  false, true,  true });
    //configs.push_back({true,  true,  false, true });
    //configs.push_back({true,  true,  true,  false});

    int seed = 0;
    while (true) {
        EmbeddingInput jittered_input = input;
        jitter_matching_vertices(jittered_input, 4, seed);
        for (const auto& config : configs) {
            // Generate random matching vertices
            Embedding em(jittered_input);

            BranchAndBoundSettings settings;
            settings.time_limit = 1 * 60;
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
                f << seed << ",";
                f << config.state_hashing << ",";
                f << config.proactive_pruning << ",";
                f << config.balanced_priority << ",";
                f << config.advanced_lower_bounds << ",";
                f << runtime << ",";
                f << last_upper_bound_event_t << ",";
                f << embedding_cost << std::endl;
            }

            std::cout << "Seed:                  " << seed << std::endl;
            std::cout << "State Hashing:         " << config.state_hashing << std::endl;
            std::cout << "Proactive Pruning:     " << config.proactive_pruning << std::endl;
            std::cout << "Balanced Priority:     " << config.balanced_priority << std::endl;
            std::cout << "Advanced Lower Bounds: " << config.advanced_lower_bounds << std::endl;
            std::cout << "Runtime:               " << runtime << std::endl;
            std::cout << "Last UB Event:         " << last_upper_bound_event_t << std::endl;
            std::cout << "Cost:                  " << embedding_cost << std::endl;
        }
        ++seed;
    }
}
