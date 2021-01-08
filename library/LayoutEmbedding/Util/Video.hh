/*
 * Author: Patrick Schmidt
 */
#pragma once

#include <filesystem>

namespace LayoutEmbedding
{

std::string format_frame_number(int _frame_number);

void render_video(
        const std::filesystem::path& _path,
        const int _fps,
        const std::string& _filename = "animation.mp4");

}
