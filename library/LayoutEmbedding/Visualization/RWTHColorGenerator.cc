#include "RWTHColorGenerator.hh"

#include <LayoutEmbedding/Visualization/RWTHColors.hh>

namespace
{
using namespace LayoutEmbedding;

static tg::color3 rwth_color_cycle[] =
{
    RWTH_BLUE_100,
    RWTH_MAGENTA_100,
    RWTH_GREEN_100,
    RWTH_RED_100,
    RWTH_PURPLE_100,
    RWTH_YELLOW_100,
    RWTH_PETROL_100,
    RWTH_ORANGE_100,
    RWTH_TEAL_100,
    RWTH_MAY_GREEN_100,
    RWTH_BORDEAUX_100,
    RWTH_LILAC_100,

    RWTH_BLUE_75,
    RWTH_MAGENTA_75,
    RWTH_GREEN_75,
    RWTH_RED_75,
    RWTH_PURPLE_75,
    RWTH_YELLOW_75,
    RWTH_PETROL_75,
    RWTH_ORANGE_75,
    RWTH_TEAL_75,
    RWTH_MAY_GREEN_75,
    RWTH_BORDEAUX_75,
    RWTH_LILAC_75,

    RWTH_BLUE_50,
    RWTH_MAGENTA_50,
    RWTH_GREEN_50,
    RWTH_RED_50,
    RWTH_PURPLE_50,
    RWTH_YELLOW_50,
    RWTH_PETROL_50,
    RWTH_ORANGE_50,
    RWTH_TEAL_50,
    RWTH_MAY_GREEN_50,
    RWTH_BORDEAUX_50,
    RWTH_LILAC_50,

    RWTH_BLUE_25,
    RWTH_MAGENTA_25,
    RWTH_GREEN_25,
    RWTH_RED_25,
    RWTH_PURPLE_25,
    RWTH_YELLOW_25,
    RWTH_PETROL_25,
    RWTH_ORANGE_25,
    RWTH_TEAL_25,
    RWTH_MAY_GREEN_25,
    RWTH_BORDEAUX_25,
    RWTH_LILAC_25,
};

template <class T, std::size_t N>
constexpr int array_size(const T (&array)[N]) noexcept
{
    return N;
}

}

namespace LayoutEmbedding {

tg::color3 RWTHColorGenerator::generate_next_color()
{
    tg::color3 result = rwth_color_cycle[current_index];
    constexpr int rwth_color_cycle_size = array_size(rwth_color_cycle);
    current_index = (current_index + 1) % rwth_color_cycle_size;
    return result;
}

std::vector<tg::color3> RWTHColorGenerator::generate_next_colors(int _n)
{
    std::vector<tg::color3> colors(_n);
    for (int i = 0; i < _n; ++i)
        colors[i] = generate_next_color();
    return colors;
}

}
