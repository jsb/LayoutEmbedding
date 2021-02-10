/**
  * Evaluates our method and greedy methods on randomly perturbed landmarks.
  *
  * Output files can be found in <build-folder>/output/jitter_evaluation_figure.
  */

#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/ApproximateGeodesicDistance.hh>
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

#include <cxxopts.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

int main(int argc, char** argv)
{
    register_segfault_handler();

    cxxopts::Options opts("jitter_evaluation_figure", "Generates Fig. 13");
    opts.add_options()("h,help", "Help");
    try {
        auto args = opts.parse(argc, argv);
        if (args.count("help")) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
    } catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path jitter_evaluation_output_dir = output_dir / "jitter_evaluation_figure";
    const fs::path jitter_evaluation_screenshots_dir = jitter_evaluation_output_dir / "screenshots";
    const fs::path jitter_evaluation_embeddings_dir = jitter_evaluation_output_dir / "embeddings";

    const bool render_jitter_trajectory = false;

    EmbeddingInput input;
    load(data_path / "models/layouts/horse_layout.obj", input.l_m, input.l_pos);
    load(data_path / "models/target-meshes/horse_8078.obj", input.t_m, input.t_pos);
    find_matching_vertices_by_proximity(input);
    //input.normalize_surface_area();
    //input.center_translation();

    std::vector<pm::vertex_handle> matching_target_vertices;
    for (const auto l_v : input.l_m.vertices()) {
        const auto& t_v = input.l_matching_vertex[l_v];
        matching_target_vertices.push_back(t_v);
    }

    auto geodesic_distance = input.l_m.vertices().make_attribute<std::vector<double>>();
    for (const auto l_v : input.l_m.vertices()) {
        const auto& t_v = input.l_matching_vertex[l_v];
        auto da = approximate_geodesic_distance(input.t_pos, t_v);
        auto d = da.to_vector();
        geodesic_distance[l_v] = d;
    }

    const fs::path stats_path = jitter_evaluation_output_dir / "stats.csv";
    fs::create_directories(jitter_evaluation_output_dir);
    {
        std::ofstream f{stats_path};
        f << "jitter_iters,seed,avg_d,max_d,algo,cost,t" << '\n';
    }

    // Shadow Screenshots
    const auto screenshot_size = tg::ivec2(1920, 1080);
    const int screenshot_samples = 64;
    const auto screenshot_view_transform = glow::viewer::camera_transform(tg::pos3(1.659233f, 0.582366f, 0.573250f), tg::pos3(1.007779f, 0.359270f, 0.402972f));
    {
        auto cfg_style = default_style();
        auto cfg_shadows_only = render_shadows_only();
        auto cfg_view = gv::config(screenshot_view_transform);

        Embedding em(input);

        fs::create_directories(jitter_evaluation_screenshots_dir);
        {
            const fs::path screenshot_path = jitter_evaluation_screenshots_dir / "layout_shadow.png";
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_layout(em);
        }
        {
            const fs::path screenshot_path = jitter_evaluation_screenshots_dir / "target_shadow.png";
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_target(em);
        }
    }

    // Uncomment to only compute selected configurations and skip the rest.
    std::set<std::tuple<int, int, std::string>> selected_configs = {
        {0, 0, "bnb"},
        {8, 0, "bnb"},
        {8, 0, "kraevoy"},
        {13, 0, "bnb"},
        {13, 0, "schreiner"},
    };

    for (int jitter_iters = 0; jitter_iters < 40; ++jitter_iters) {
        int num_seeds = 1;
        if (jitter_iters == 0) {
            num_seeds = 1;
        }
        for (int seed = 0; seed < num_seeds; ++seed) {
            EmbeddingInput jittered_input = input; // Copy
            jitter_matching_vertices(jittered_input, jitter_iters, seed);

            if (render_jitter_trajectory) {
                auto cfg_style = default_style();
                auto cfg_view = gv::config(glow::viewer::camera_transform(tg::pos3(0.493576f, 0.350499f, 0.702366f), tg::pos3(-0.141555f, 0.088053f, 0.878121f)));
                Embedding em(jittered_input);

                const fs::path screenshot_path = jitter_evaluation_screenshots_dir / ("jitter_" + std::to_string(jitter_iters) + ".png");
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                {
                    auto v = gv::view();
                    view_target_mesh(em);
                    auto l_v = em.layout_mesh().vertices()[50]; // One of the layout vertices on the horse's nose
                    auto t_v = em.matching_target_vertex(l_v);
                    view_vertex(em.target_pos(), t_v, RWTH_RED, 20.0f);
                }
            }

            double max_d = 0.0;
            double sum_d = 0.0;
            for (const auto l_v : jittered_input.l_m.vertices()) {
                const auto& t_v = jittered_input.l_matching_vertex[l_v];
                const auto d = geodesic_distance[l_v.idx][t_v.idx.value];
                max_d = std::max(max_d, d);
                sum_d += d;
            }
            const double avg_d = sum_d / jittered_input.l_m.vertices().size();

            std::cout<< "Jitter Distance: " << jitter_iters << ", Seed: " << seed << " Avg Geodesic Distance: " << avg_d << ", Max Geodesic Distance: " << max_d << std::endl;

            const std::vector<std::string> algos = {
                "bnb",
                "praun",
                "kraevoy",
                "schreiner",
            };

            for (const auto& algo : algos) {
                if (!selected_configs.empty()) {
                    const auto config = std::tuple<int, int, std::string>(jitter_iters, seed, algo);
                    if (selected_configs.count(config) == 0) {
                        continue;
                    }
                }

                Embedding em(jittered_input);

                double cost = std::numeric_limits<double>::infinity();
                glow::timing::CpuTimer timer;
                if (algo == "bnb") {
                    BranchAndBoundSettings settings;
                    settings.time_limit = 60.0;
                    auto result = branch_and_bound(em, settings);
                    cost = result.cost;
                }
                else if (algo == "praun") {
                    auto result = embed_praun(em);
                    cost = result.cost;
                }
                else if (algo == "kraevoy") {
                    auto result = embed_kraevoy(em);
                    cost = result.cost;
                }
                else if (algo == "schreiner") {
                    auto result = embed_schreiner(em);
                    cost = result.cost;
                }
                else {
                    LE_ERROR_THROW("unknown algo");
                }
                const double runtime = timer.elapsedSecondsD();

                {
                    std::ofstream f{stats_path, std::ofstream::app};
                    f << jitter_iters << ",";
                    f << seed << ",";
                    f << avg_d << ",";
                    f << max_d << ",";
                    f << algo << ",";
                    f << cost << ",";
                    f << runtime << "\n";
                }

                // Smooth embedding
                const auto em_smoothed = smooth_paths(em, 1);

                // Screenshots
                std::string filename_prefix = std::to_string(jitter_iters) + "_" + std::to_string(seed) + "_" + algo;
                {
                    auto cfg_style = default_style();
                    auto cfg_objects_only = render_objects_only();
                    auto cfg_view = gv::config(screenshot_view_transform);

                    fs::create_directories(jitter_evaluation_screenshots_dir);
                    {
                        const fs::path screenshot_path = jitter_evaluation_screenshots_dir / (filename_prefix + "_target.png");
                        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                        view_target(em);
                    }
                    {
                        const fs::path screenshot_path = jitter_evaluation_screenshots_dir / (filename_prefix + "_target_smoothed.png");
                        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
                        view_target(em_smoothed);
                    }
                }

                // Save to file
                fs::create_directories(jitter_evaluation_embeddings_dir);
                {
                    fs::path embedding_path = jitter_evaluation_embeddings_dir / filename_prefix;
                    em.save(embedding_path);
                }
                {
                    fs::path embedding_path = jitter_evaluation_embeddings_dir / (filename_prefix + "_smoothed");
                    em_smoothed.save(embedding_path);
                }
            }

        }
    }
}
