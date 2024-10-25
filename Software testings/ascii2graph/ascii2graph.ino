/**
 * Remove doctest.h and test_ascii2graph.cpp when uploading to Arduino.
 */

#include "Graph.hpp"
#include "ascii2graph.hpp"

// Helper function.
void insert_edge(mtrn3100::Graph<int, int>& maze, int cell1, int cell2) {
    // Graph is a directed graph so need to do both directions.
    maze.insert_edge(cell1, cell2, 0);
    maze.insert_edge(cell2, cell1, 0);
}

void setup() {
    Serial.begin(9600);

    // This is how the nodes are numbered.
    //  --- --- --- --- --- --- --- --- ---
    // | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
    //  --- --- --- --- --- --- --- --- ---
    // | 10| 11| 12| 13| 14| 15| 16| 17| 18|
    //  --- --- --- --- --- --- --- --- ---
    // | 19| 20| 21| 22| 23| 24| 25| 26| 27|
    //  --- --- --- --- --- --- --- --- ---
    // | 28| 29| 30| 31| 32| 33| 34| 35| 36|
    //  --- --- --- --- --- --- --- --- ---
    // | 37| 38| 39| 40| 41| 42| 43| 44| 45|
    //  --- --- --- --- --- --- --- --- ---
    char maze[] =" --- --- --- --- --- --- --- --- --- \n|       |   |           |           |\n     ---     --- ---         ---     \n|   |       |           |       |   |\n         --- ---             ---     \n|                   |   |       |   |\n         --- --- ---     ---     --- \n|           |                   |   |\n ---     --- --- ---     ---         \n|               |           |       |\n --- --- --- --- --- --- --- --- --- \0";

    int mazeSize = sizeof(" --- --- --- --- --- --- --- --- --- \n|       |   |           |           |\n     ---     --- ---         ---     \n|   |       |           |       |   |\n         --- ---             ---     \n|                   |   |       |   |\n         --- --- ---     ---     --- \n|           |                   |   |\n ---     --- --- ---     ---         \n|               |           |       |\n --- --- --- --- --- --- --- -");     
    while(1){Serial.println(mazeSize);}
    // point: row, col.
    int p11{1}, p12{2}, p13{3}, p14{4}, p15{5}, p16{6}, p17{7}, p18{8}, p19{9};
    int p21{10}, p22{11}, p23{12}, p24{13}, p25{14}, p26{15}, p27{16}, p28{17}, p29{18};
    int p31{19}, p32{20}, p33{21}, p34{22}, p35{23}, p36{24}, p37{25}, p38{26}, p39{27};
    int p41{28}, p42{29}, p43{30}, p44{31}, p45{32}, p46{33}, p47{34}, p48{35}, p49{36};
    int p51{37}, p52{38}, p53{39}, p54{40}, p55{41}, p56{42}, p57{43}, p58{44}, p59{45};

    mtrn3100::Graph<int, int> expected(p11, p12, p13, p14, p15, p16, p17, p18, p19, p21, p22, p23, p24, p25, p26, p27,
                                       p28, p29, p31, p32, p33, p34, p35, p36, p37, p38, p39, p41, p42, p43, p44, p45,
                                       p46, p47, p48, p49, p51, p52, p53, p54, p55, p56, p57, p58, p59);

    // All the vertical connections.
    insert_edge(expected, p11, p21);
    insert_edge(expected, p21, p31);
    insert_edge(expected, p31, p41);

    insert_edge(expected, p22, p32);
    insert_edge(expected, p32, p42);
    insert_edge(expected, p42, p52);

    insert_edge(expected, p13, p23);

    insert_edge(expected, p25, p35);

    insert_edge(expected, p16, p26);
    insert_edge(expected, p26, p36);
    insert_edge(expected, p36, p46);
    insert_edge(expected, p46, p56);

    insert_edge(expected, p17, p27);
    insert_edge(expected, p27, p37);

    insert_edge(expected, p38, p48);
    insert_edge(expected, p48, p58);

    insert_edge(expected, p19, p29);
    insert_edge(expected, p29, p39);
    insert_edge(expected, p49, p59);

    // All the horizontal connections.
    insert_edge(expected, p11, p12);
    insert_edge(expected, p14, p15);
    insert_edge(expected, p15, p16);
    insert_edge(expected, p17, p18);
    insert_edge(expected, p18, p19);

    insert_edge(expected, p22, p23);
    insert_edge(expected, p24, p25);
    insert_edge(expected, p25, p26);
    insert_edge(expected, p27, p28);

    insert_edge(expected, p31, p32);
    insert_edge(expected, p32, p33);
    insert_edge(expected, p33, p34);
    insert_edge(expected, p34, p35);
    insert_edge(expected, p37, p38);

    insert_edge(expected, p41, p42);
    insert_edge(expected, p42, p43);
    insert_edge(expected, p44, p45);
    insert_edge(expected, p45, p46);
    insert_edge(expected, p46, p47);
    insert_edge(expected, p47, p48);

    insert_edge(expected, p51, p52);
    insert_edge(expected, p52, p53);
    insert_edge(expected, p53, p54);
    insert_edge(expected, p55, p56);
    insert_edge(expected, p56, p57);
    insert_edge(expected, p58, p59);

    auto const actual = mtrn3100::ascii2graph(maze);
    for (auto e : actual.edges()) {
        int src = mtrn3100::get<0>(e.value);
        int dst = mtrn3100::get<1>(e.value);
        Serial.print(src);
        Serial.print(" -- ");
        Serial.println(dst);
    }
}

void loop() {}