/*
 * Author: Janis Born
 */
#pragma once

#include <filesystem>
#include <string>

namespace LayoutEmbedding
{

struct AnnotationOptions
{
    std::string corner = "NorthWest";
    std::string color = "black";
    int size = 26;
};

/// Prints the text _caption onto an existing image, given by _path.
/// Uses ImageMagick's "convert" CLI.
void annotate_image(
        const std::filesystem::path& _path,
        const std::string& _caption,
        const AnnotationOptions& _options = AnnotationOptions());

}
