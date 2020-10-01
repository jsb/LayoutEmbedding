#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080);
const int screenshot_samples = 64;
const auto cam_pos = glow::viewer::camera_transform(tg::pos3(-0.460784f, 0.451778f, 0.941845f), tg::pos3(-0.291428f, 0.296843f, 0.611157f));

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;


    const auto layout_path = fs::path(LE_DATA_PATH) / "models/layouts/human/face_disk2.obj";
    const auto target_path = fs::path(LE_DATA_PATH) / "models/target-meshes/human/face_disk.obj";

    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "disk_figure";
    fs::create_directories(output_dir);

    EmbeddingInput input;
    input.load(layout_path, target_path);

    Embedding em(input);
    embed_greedy(em);

    em = smooth_paths(em, 1, true);

    // Embedding screenshot
    {
        auto cfg_style = default_style();
        auto cfg_view = gv::config(cam_pos);
        const auto screenshot_path = output_dir / "face_disk.png";
//        auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));

        {
            auto g = gv::grid();
            view_layout(em, true, 5, 5);
            view_target(em, true, 5, 5);
        }
    }


//    // Paths
//    const auto embedding_base_dir = fs::path(LE_DATA_PATH) / "embeddings/homotopy_cube";
//    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "homotopy_cube";
//    fs::create_directories(output_dir);

//    std::vector<std::string> names
//    {
//        "embedding_original",
//        "embedding_twist1",
//        "embedding_twist2",
//    };

//    for (const auto name : names)
//    {
//        const auto embedding_path = embedding_base_dir / name / "homotopy_cube";

//        EmbeddingInput input;
//        Embedding em(input);
//        em.load(embedding_path);

//        if (name == "embedding_twist1")
//        {
//            // Smooth paths in y direction
//            std::vector<pm::edge_handle> l_e_smooth;
//            for (auto e : em.layout_mesh().edges())
//            {
//                if (fabs(tg::dot(tg::normalize(pm::edge_dir(e.halfedgeA(), em.layout_pos())), tg::vec3(0, 1, 0))) > 0.5)
//                    l_e_smooth.push_back(e);
//            }
//            em = smooth_paths(em, l_e_smooth, 4, false);
//        }
//        else if (name == "embedding_twist2")
//        {
//            std::vector<pm::edge_handle> l_e_smooth;
//            for (auto e : em.layout_mesh().edges())
//            {
//                if (e != pm::halfedge_from_to(em.layout_mesh().vertices()[3], em.layout_mesh().vertices()[7]).edge() &&
//                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[2], em.layout_mesh().vertices()[0]).edge() &&
//                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[0], em.layout_mesh().vertices()[1]).edge() &&
//                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[1], em.layout_mesh().vertices()[5]).edge())
//                {
//                    l_e_smooth.push_back(e);
//                }
//            }

//            em = subdivide(em);
//            em = smooth_paths(em, l_e_smooth, 1, false);
//            em = subdivide(em);
//            em = smooth_paths(em, l_e_smooth, 1, false);
//        }

//        // Layout screenshot
//        {
//            auto cfg_style = default_style();
//            auto cfg_view = gv::config(cam_pos);
//            const auto screenshot_path = output_dir / "layout.png";
//            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
//            view_layout(em, true, 30, 15);
//        }

//        // Embedding screenshot
//        {
//            auto cfg_style = default_style();
//            auto cfg_view = gv::config(cam_pos);
//            const auto screenshot_path = output_dir / (name + ".png");
//            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
//            view_target(em, true, 30, 15);
//        }
//    }
}
