#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "Graph.hpp"
#include "doctest.h"
#include "graph2ascii.hpp"

// This is how the nodes are numbered.
//  --- --- ---
// | 1 | 2 | 3 |
//  --- --- ---
// | 4 | 5 | 6 |
//  --- --- ---
// | 7 | 8 | 9 |
//  --- --- ---

namespace {

// Helper function.
void insert_edge(mtrn3100::Graph<int, int>& maze, int cell1, int cell2) {
    // Graph is a directed graph so need to do both directions.
    maze.insert_edge(cell1, cell2, 0);
    maze.insert_edge(cell2, cell1, 0);
}

}  // namespace

TEST_CASE("Maze 1.") {
    mtrn3100::Graph<int, int> maze(1, 2, 3, 4, 5, 6, 7, 8, 9);
    insert_edge(maze, 1, 2);
    insert_edge(maze, 1, 4);
    insert_edge(maze, 4, 7);
    insert_edge(maze, 7, 8);
    insert_edge(maze, 8, 9);
    insert_edge(maze, 5, 8);
    insert_edge(maze, 5, 6);
    insert_edge(maze, 3, 6);

    CHECK(strcmp(mtrn3100::graph2ascii(maze),
                 " --- --- --- \n"
                 "|       |   |\n"
                 "     ---     \n"
                 "|   |       |\n"
                 "         --- \n"
                 "|           |\n"
                 " --- --- --- \0") == 0);
}

TEST_CASE("Maze 2.") {
    mtrn3100::Graph<int, int> maze(1, 2, 3, 4, 5, 6, 7, 8, 9);
    insert_edge(maze, 1, 2);
    insert_edge(maze, 1, 4);
    insert_edge(maze, 2, 3);
    insert_edge(maze, 2, 5);
    insert_edge(maze, 3, 6);
    insert_edge(maze, 4, 7);
    insert_edge(maze, 7, 8);
    insert_edge(maze, 8, 9);

    CHECK(strcmp(mtrn3100::graph2ascii(maze),
                 " --- --- --- \n"
                 "|           |\n"
                 "             \n"
                 "|   |   |   |\n"
                 "     --- --- \n"
                 "|           |\n"
                 " --- --- --- \0") == 0);
}

TEST_CASE("Maze 3.") {
    mtrn3100::Graph<int, int> maze(1, 2, 3, 4, 5, 6, 7, 8, 9);
    insert_edge(maze, 1, 2);
    insert_edge(maze, 2, 3);
    insert_edge(maze, 1, 4);
    insert_edge(maze, 3, 6);
    insert_edge(maze, 4, 5);
    insert_edge(maze, 5, 6);
    insert_edge(maze, 4, 7);
    insert_edge(maze, 6, 9);
    insert_edge(maze, 7, 8);
    insert_edge(maze, 8, 9);

    CHECK(strcmp(mtrn3100::graph2ascii(maze),
                 " --- --- --- \n"
                 "|           |\n"
                 "     ---     \n"
                 "|           |\n"
                 "     ---     \n"
                 "|           |\n"
                 " --- --- --- \0") == 0);
}