#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "Graph.hpp"
#include "LinkedList.hpp"
#include "doctest.h"
#include "shortest_path.hpp"

TEST_CASE("No shortest path exists.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    auto paths = mtrn3100::bfs_multiple(g, 1, 2);
    CHECK(paths.size() == 0);
}

TEST_CASE("Single shortest path.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    g.insert_edge(1, 2, 1);
    g.insert_edge(2, 3, 1);
    g.insert_edge(3, 6, 1);
    g.insert_edge(6, 4, 1);
    g.insert_edge(4, 12, 1);
    auto paths = mtrn3100::bfs_multiple(g, 1, 12);
    CHECK(paths.size() == 1);

    auto path = paths[0];
    CHECK(path.size() == 6);
    CHECK(path[0] == 1);
    CHECK(path[1] == 2);
    CHECK(path[2] == 3);
    CHECK(path[3] == 6);
    CHECK(path[4] == 4);
    CHECK(path[5] == 12);
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

    auto paths = mtrn3100::bfs_multiple(g, 1, 12);
    CHECK(paths.size() == 2);

    {
        auto path = paths[0];
        CHECK(path.size() == 6);
        CHECK(path[0] == 1);
        CHECK(path[1] == 2);
        CHECK(path[2] == 3);
        CHECK(path[3] == 6);
        CHECK(path[4] == 4);
        CHECK(path[5] == 12);
    }

    {
        auto path = paths[1];
        CHECK(path.size() == 6);
        CHECK(path[0] == 1);
        CHECK(path[1] == 11);
        CHECK(path[2] == 8);
        CHECK(path[3] == 9);
        CHECK(path[4] == 5);
        CHECK(path[5] == 12);
    }
}

TEST_CASE("Multiple possible paths with single shortest path.") {
    mtrn3100::Graph<int, int> g(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);

    // Path 1.
    g.insert_edge(1, 2, 1);
    g.insert_edge(2, 3, 1);
    g.insert_edge(3, 6, 1);
    g.insert_edge(6, 4, 1);
    g.insert_edge(4, 12, 1);

    // Path 2.
    g.insert_edge(1, 11, 1);
    g.insert_edge(11, 8, 1);
    g.insert_edge(8, 6, 1);
    g.insert_edge(6, 12, 1);

    // Path 3.
    g.insert_edge(1, 7, 1);
    g.insert_edge(7, 4, 1);
    g.insert_edge(4, 12, 1);

    auto paths = mtrn3100::bfs_multiple(g, 1, 12);
    CHECK(paths.size() == 1);

    auto path = paths[0];
    CHECK(path.size() == 4);
    CHECK(path[0] == 1);
    CHECK(path[1] == 7);
    CHECK(path[2] == 4);
    CHECK(path[3] == 12);
}