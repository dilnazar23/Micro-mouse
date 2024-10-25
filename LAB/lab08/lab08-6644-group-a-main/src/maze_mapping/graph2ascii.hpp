#pragma once

#include "Graph.hpp"
//#include <iostream>
namespace mtrn3100 {

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
char* graph2ascii(Graph<N, E> const& g) {
    // 3 x 3 maze.
    int const numRows = 3;
    int const numCols = 3;

    // Helper function to convert between (rows, cols) and number of chars.
    auto row2charPos = [](int const r) { return r * 2 + 1; };
    auto col2charPos = [](int const c) { return c * 4 + 2; };

    int const numCharRows = row2charPos(numRows);      // 7 characters.
    int const numCharCols = col2charPos(numCols);      // 14 characters.
    char* maze = new char[numCharCols * numCharRows];  // 98 bytes is needed to represent 3 x 3 maze.

    // Helper function to access the maze with a 2D model.
    auto maze2d = [&maze, &numCharCols](unsigned const r, unsigned const c) -> char& {
        return maze[r * numCharCols + c];
    };

    // Initialise the maze values.
    for (int i = 0; i < numCharCols * numCharRows; i++) {
        maze[i] = ' ';
    }

    // Do new lines.
    for (int r = 0; r < numCharRows; r++) {
        maze2d(r, numCharCols - 1) = '\n';
    }

    // Terminate the string.
    maze2d(numCharRows - 1, numCharCols - 1) = '\0';

    // Do external walls.
    // horizontal external wall
    for (int i = 0; i < numCharRows; i++){
        if (i%2 ==0 ){
            for (int j = 0; j < numCharCols-1; j++){
                if (j % 4 !=0){
                    maze2d(i,j) = '-';
                }else{
                    maze2d(i,j) = ' ';
                }
            }
        }else {
            for (int j = 0; j < numCharCols-1; j++){
                if (j % 4 ==0){
                    maze2d(i,j) = '|';
                }   
            }
            //maze2d(i,0) = '|';
            //maze2d(i,numCharCols-2) = '|';
        }
    }

    // Do internal walls.
    for (auto const& edge : g.edges()) {
        auto const& src=get<0>(edge.value);
        auto const& dis=get<1>(edge.value);
        if (src > dis){continue;}
        //std::cout<< "Source: " << src << ", Destination: " << dis << std::endl;
        int src_col =0; 
        int src_row =0; 
        int dis_row = 0;
        if ((src<4) && (src>0)){
            src_row = 1;
        }else if((src<7) && (src>3)){
            src_row = 3;
        }else if((src<10) && (src>6)){
            src_row = 5;
        }
        if ((dis<4) && (dis>0)){
            dis_row = 1;
        }else if((dis<7) && (dis>3)){
            dis_row = 3;
        }else if((dis<10) && (dis>6)){
            dis_row = 5;
        }
        if (src%3 == 0){
            src_col = 12;            
        }else{
            src_col = src%3 *4;
        }
        //std::cout<< "Source: " << src_row <<','<<src_col << ", Destination: " << dis_row << std::endl;
        if (src_row==dis_row){
            maze2d(src_row,src_col) = ' ';
            //std::cout << maze << std::endl;
        }else{
            int diff = dis_row-src_row;
            for (int k = src_col-3;k <src_col;k++){
                maze2d(src_row+1,k) = ' ';
                }
            if (diff != 2){
                for (int k = src_col-3;k <src_col;k++){
                maze2d(src_row+3,k) = ' ';
                }
            }        
            //std::cout << maze << std::endl;
        }
                
         
    }
    
     //std::cout << maze << std::endl;

    return maze;
}

}  // namespace mtrn3100