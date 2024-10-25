#include "Graph.hpp"
#include "LinkedList.hpp"

namespace mtrn3100 {

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
bool bfs_simple(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<N> queue;
    queue.push_front(src);
    return false;
}

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
LinkedList<N> bfs_single(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<LinkedList<N>> queue;
    queue.push_front({src});
    return {};
}

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
LinkedList<LinkedList<N>> bfs_multiple(Graph<N, E> const& g, N const& src, N const& dst) {
    LinkedList<LinkedList<N>> paths;
    LinkedList<LinkedList<N>> queue;
    queue.push_front({src});
    return paths;
}

}  // namespace mtrn3100
