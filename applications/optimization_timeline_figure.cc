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

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;

    register_segfault_handler();

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path optimization_timeline_output_dir = output_dir / "optimization_timeline";

    const int seed = 281; // Produces a visually interesting sequence of upper and lower bound events.
    std::cout << "Seed: " << seed << std::endl;
    const fs::path cached_embedding_path = optimization_timeline_output_dir / (std::to_string(seed) + ".lem");
    const fs::path cached_embedding_path_without_extension = optimization_timeline_output_dir / std::to_string(seed);

    EmbeddingInput input;
    load(data_path / "models/target-meshes/sphere.obj", input.t_m, input.t_pos);
    load(data_path / "models/layouts/cube_layout.obj", input.l_m, input.l_pos);
    std::srand(seed);
    randomize_matching_vertices(input);

    Embedding em(input);
    if (fs::exists(cached_embedding_path)) {
        std::cout << "Loading cached embedding " << cached_embedding_path << " ..." << std::endl;
        em.load(cached_embedding_path_without_extension);
    }
    else {
        std::cout << "Computing embedding ..." << std::endl;

        BranchAndBoundSettings settings;
        settings.use_greedy_init = false;
        settings.record_lower_bound_events = true;
        settings.record_upper_bound_events = true;
        settings.time_limit = 5 * 60;
        auto result = branch_and_bound(em, settings);

        const fs::path lb_path = optimization_timeline_output_dir / (std::to_string(seed) + "_lower_bound.csv");
        const fs::path ub_path = optimization_timeline_output_dir / (std::to_string(seed) + "_upper_bound.csv");

        fs::create_directories(optimization_timeline_output_dir);
        {
            std::ofstream f{lb_path};
            f << "t,lower_bound" << '\n';
            for (const auto& record : result.lower_bound_events) {
                f << record.t << "," << record.lower_bound << '\n';
            }
        }
        {
            std::ofstream f{ub_path};
            f << "t,upper_bound" << '\n';
            for (const auto& record : result.upper_bound_events) {
                f << record.t << "," << record.upper_bound << '\n';
            }
        }

        std::cout << "Saving embedding " << cached_embedding_path << " ..." << std::endl;
        em.save(cached_embedding_path_without_extension);
    }
}
