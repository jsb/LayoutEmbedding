#include "Parametrization.hh"

#include <LayoutEmbedding/Util/Assert.hh>
#include <LayoutEmbedding/ExactPredicates.h>

namespace LayoutEmbedding
{

namespace
{

const double* ptr(
        const tg::dpos2& _p)
{
    return &_p.x;
}

}

bool injective(
        const VertexParam& _param)
{
    exactinit();

    for (auto f : _param.mesh().faces())
    {
        LE_ASSERT(f.vertices().size() == 3);
        auto it = f.vertices().begin();
        const auto a = _param[*it];
        ++it;
        const auto b = _param[*it];
        ++it;
        const auto c = _param[*it];

        if (orient2d(ptr(a), ptr(b), ptr(c)) <= 0.0)
            return false;
    }

    return true;
}

}
