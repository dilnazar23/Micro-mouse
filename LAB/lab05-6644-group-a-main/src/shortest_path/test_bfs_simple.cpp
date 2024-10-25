#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "Graph.hpp"
#include "LinkedList.hpp"
#include "doctest.h"
#include "shortest_path.hpp"

TEST_CASE("No shortest path exists.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    CHECK(mtrn3100::bfs_simple(g, 1, 2) == false);
}

TEST_CASE("Single shortest path.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    g.insert_edge(1, 2, 1);
    g.insert_edge(2, 3, 1);
    g.insert_edge(3, 6, 1);
    g.insert_edge(6, 4, 1);
    g.insert_edge(4, 12, 1);
    CHECK(mtrn3100::bfs_simple(g, 1, 12) == true);
}

TEST_CASE("Multiple shortest path.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);

    // First path.
    g.insert_edge(1, 2, 1);
    g.insert_edge(2, 3, 1);
    g.insert_edge(3, 6, 1);
    g.insert_edge(6, 4, 1);
    g.insert_edge(4, 12, 1);

    // Second path.
    g.insert_edge(1, 11, 1);
    g.insert_edge(11, 8, 1);
    g.insert_edge(8, 9, 1);
    g.insert_edge(9, 5, 1);
    g.insert_edge(5, 12, 1);

    CHECK(mtrn3100::bfs_simple(g, 1, 12) == true);
}
