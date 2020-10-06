/**
  * Embeds an automatically generated layout into each selected model
  * of the SHREC07 datset.
  *
  * Instructions:
  *
  *     * Run shrec07_generate_layouts before running this file.
  *     * Running this experiment might take a while (~24h).
  *
  * Output files can be found in <build-folder>/output/shrec07_results.
  *
  */

#include "shrec07.hh"

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Util/Assert.hh>
#include <LayoutEmbedding/Util/StackTrace.hh>
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
        "praun",
        "kraevoy",
        "schreiner",
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

        if (algorithm == "praun") {
            embed_praun(em);
        }
        else if (algorithm == "kraevoy") {
            embed_kraevoy(em);
        }
        else if (algorithm == "schreiner") {
            embed_schreiner(em);
        }
        else if (algorithm == "bnb") {
            BranchAndBoundSettings settings;
            settings.time_limit = 5 * 60;
            auto result = branch_and_bound(em, settings);

            double last_upper_bound_event_t = std::numeric_limits<double>::infinity();
            if (!result.upper_bound_events.empty()) {
                const auto& last_upper_bound_event = result.upper_bound_events.back();
                last_upper_bound_event_t = last_upper_bound_event.t;
            }

            const fs::path bnb_stats_path = shrec_results_dir / ("stats_" + _name + "_bnb.csv");
            std::ofstream f{bnb_stats_path};
            f << "gap,lower_bound,last_upper_bound_event_t,max_state_tree_memory" << std::endl;
            f << result.gap << "," << result.lower_bound << "," << last_upper_bound_event_t << "," << result.max_state_tree_memory_estimate << std::endl;
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

        // Smooth embedding
        const auto em_smoothed = smooth_paths(em, 1);

        // Visualization
        auto cfg_style = gv::config(gv::no_grid, gv::no_outline, gv::background_color(RWTH_WHITE));

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
            {
                const fs::path screenshot_path = shrec_results_dir / ("screenshot_" + _name + "_" + algorithm + "_target_smoothed.png");
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string()));
                view_target(em_smoothed);
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
            {
                view_target(em_smoothed);
            }
        }

        // Save to file
        fs::path saved_embeddings_dir = shrec_results_dir / "saved_embeddings";
        fs::create_directories(saved_embeddings_dir);
        {
            fs::path embedding_path = saved_embeddings_dir / (_name + "_" + algorithm);
            em.save(embedding_path);
        }
        {
            fs::path embedding_path = saved_embeddings_dir / (_name + "_" + algorithm + "_smoothed");
            em_smoothed.save(embedding_path);
        }
    }
}

}

int main()
{
    namespace fs = std::filesystem;

    register_segfault_handler();

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
            if (!input.load(layout_mesh_path, target_mesh_path, corrs_path, LandmarkFormat::id_x_y_z)) {
                continue;
            }

            if (shrec_flipped_landmarks.count(mesh_id)) {
                std::cout << "This object is flipped. Inverting layout mesh." << std::endl;
                input.invert_layout();
            }

            input.normalize_surface_area();
            input.center_translation();
            compute_embeddings(std::to_string(mesh_id), input);
        }
    }
}
