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

    EmbeddingInput input;
    load(data_path / "models/target-meshes/sphere.obj", input.t_m, input.t_pos);
    load(data_path / "models/layouts/cube_layout.obj", input.l_m, input.l_pos);

    for (int seed = 0; seed < 1000; ++seed) {
        std::cout << "Seed: " << seed << std::endl;

        const fs::path lb_path = optimization_timeline_output_dir / (std::to_string(seed) + "_lower_bound.csv");
        const fs::path ub_path = optimization_timeline_output_dir / (std::to_string(seed) + "_upper_bound.csv");

        // Generate random matching vertices
        std::srand(seed); // TODO: make the seed a parameter of randomize_matching_vertices?
        randomize_matching_vertices(input);
        Embedding em(input);

        BranchAndBoundSettings settings;
        settings.record_lower_bound_events = true;
        settings.record_upper_bound_events = true;
        settings.time_limit = 10 * 60;
        auto result = branch_and_bound(em, settings);

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
    }
}
