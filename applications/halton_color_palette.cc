#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/StackTrace.hh>
#include <LayoutEmbedding/Visualization/HaltonColorGenerator.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;
    register_segfault_handler();

    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path palette_path = output_dir / "HaltonColors.gpl";

    const int n_colors = 100;

    fs::create_directories(output_dir);
    std::ofstream f(palette_path);
    f << "GIMP Palette" << '\n';
    f << "Name: Halton Colors" << '\n';
    f << "#" << '\n';

    HaltonColorGenerator hcg;
    for (int i = 0; i < n_colors; ++i) {
        const auto& color = hcg.generate_next_color();
        const int r = static_cast<int>(color.r * 255.0f);
        const int g = static_cast<int>(color.g * 255.0f);
        const int b = static_cast<int>(color.b * 255.0f);
        f << r << " ";
        f << g << " ";
        f << b << " ";
        f << "Halton Color " << i << '\n';
    }
}
