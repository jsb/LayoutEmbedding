/*
 * Author: Janis Born
 */
#include "AnnotateImage.hh"

#include <LayoutEmbedding/Util/Assert.hh>

namespace
{

std::string escape_string(const std::string& _input)
{
    std::string output;
    for (const char c : _input)
    {
        if (c == '"')
            output += "\\\"";
        else if (c == '\\')
            output += "\\\\";
        else if (c == '\n')
            output += "\\n";
        else if (c == '\t')
            output += "\\t";
        else if (c == '\r')
            output += "\\r";
        else if (c == '%')
            output += "%%";
        else if (std::isprint(c))
            output += c;
        else
        {
            output += c;
            /*
            constexpr char const* const hex_digits = "0123456789ABCDEF";
            output += "\\x";
            output += hex_digits[(c & 0xF0) >> 4];
            output += hex_digits[(c & 0x0F) >> 0];
            */
        }
    }
    return output;
}

}

namespace LayoutEmbedding
{

void annotate_image(
        const std::filesystem::path& _path,
        const std::string& _caption,
        const AnnotationOptions& _options)
{
    const std::string command = "convert " +
            _path.string() + " " +
            "-gravity " + _options.corner + " " +
            "-pointsize " + std::to_string(_options.size) + " " +
            "-fill " + _options.color + " " +
            "-annotate 0 \"" + escape_string(_caption) + "\" " +
            _path.string();

    std::system(command.c_str());
}

}
