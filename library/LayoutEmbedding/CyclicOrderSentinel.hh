#pragma once

#include <algorithm>
#include <initializer_list>

namespace LayoutEmbedding {

// A helper object to detect whether a sequence of occurences is compatible with a certain cyclic order:

template <typename T>
class CyclicOrderSentinel
{
public:
    explicit CyclicOrderSentinel(const std::initializer_list<T>& _items) :
        items(_items)
    {
    }

    void encounter(const T& _item)
    {
        auto it = std::find(items.begin(), items.end(), _item);
        if (it == items.end()) {
            return;
        }

        const int index = std::distance(items.begin(), it);

        if (current_index < 0) {
            // prev_index not initialized yet. (This was the first encountered element.)
            current_index = index;
        }
        else {
            const int expected_next_index = (current_index + 1) % items.size();
            if (index != expected_next_index) {
                good = false;
            }
            current_index = index;
        }
    }

    bool valid() const
    {
        return good;
    }

private:
    bool good = true;
    std::initializer_list<T> items;
    int current_index = -1;
};

}
