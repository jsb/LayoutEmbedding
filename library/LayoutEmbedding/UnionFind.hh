#pragma once

#include <LayoutEmbedding/Util/Assert.hh>

#include <numeric>
#include <vector>

namespace LayoutEmbedding {

struct UnionFind
{
    explicit UnionFind(int num_items = 0)
    {
        reset(num_items);
    }

    void reset(int num_items)
    {
        parents.resize(num_items);
        std::iota(parents.begin(), parents.end(), 0);
    }

    void clear()
    {
        reset(0);
    }

    int add()
    {
        parents.push_back(parents.size());
        return parents.back();
    }

    int representative(int x)
    {
        LE_ASSERT_GEQ(x, 0);
        LE_ASSERT_L(x, parents.size());
        if (x != parents[x]) {
            parents[x] = representative(parents[x]);
        }
        return parents[x];
    }

    int representative(int x) const
    {
        LE_ASSERT_GEQ(x, 0);
        LE_ASSERT_L(x, parents.size());
        if (x == parents[x]) {
            return x;
        }
        else {
            return representative(parents[x]);
        }
    }

    void merge(int x, int y)
    {
        int rootx = representative(x);
        int rooty = representative(y);
        parents[rootx] = rooty;
    }

    bool equivalent(int x, int y)
    {
        return representative(x) == representative(y);
    }

    bool equivalent(int x, int y) const
    {
        return representative(x) == representative(y);
    }

    std::vector<int> parents;
};

}
