#pragma once

namespace LayoutEmbedding {

// A hack to get at the container enclosed in a std::queue or std::priority queue.
// https://stackoverflow.com/a/29325258/3077540
template <class Queue>
typename Queue::container_type& get_container(Queue& a)
{
    struct HackedQueue : Queue
    {
        static typename Queue::container_type& get(Queue& a)
        {
            return a.*&HackedQueue::c;
        }
    };
    return HackedQueue::get(a);
}

}
