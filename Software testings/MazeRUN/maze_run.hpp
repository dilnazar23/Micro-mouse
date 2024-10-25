#pragma once

#include "Graph.hpp"
#include "Tuple.hpp"
#include "LinkedList.hpp"

// #include <iostream>

// Takes in a node and return a tuple with the node's corresponding row and col value
mtrn3100::Tuple<int,int> nodetorowcol (int node) {
    int counter = 0;
    int remainder = node;
    mtrn3100::Tuple<int,int> rowcol;
    while (true) {
        if (remainder <= 9) {
            break;
        }
        remainder = remainder - 9;
        counter++;
    }
    mtrn3100::get<0>(rowcol) = counter;
    mtrn3100::get<1>(rowcol) = remainder - 1;
    return rowcol;
};

// Ensures heading is always between -PI/2 and PI
float correctHead(float initial) {
    if (abs(initial - 3*M_PI/2) < 0.01) { //3pi/2 correction
        return -M_PI/2;
    } else if (abs((initial + M_PI)) < 0.01) { //-pi correction
        return M_PI;
    } else if (abs(initial) < 0.01) {
        return 0.0;
    } else {
        return initial;
    }
};



// Takes in previous, current and next node and outputs the desired heading to move to the next node
float nodetodirection (int c, int n , float previous) {
    
    if (n - c == -9) {
        return 0.0; //NORTH
    } else if (n - c == 1) {
        return M_PI/2; //EAST
    } else if (n - c == 9) {
        return M_PI; //SOUTH
    } else if (n - c == -1) {
        return -M_PI/2; //WEST
    } else if (n - c == 0) {
        return previous;
    } else {
        return correctHead(previous);
    };
}

// create a turn right and turn left function 
// float turnRight ()
float turnRight(float previous_heading) {
    float new_heading = previous_heading + M_PI/2;
    new_heading = correctHead(new_heading);
    return new_heading;
}

float turnLeft(float previous_heading) {
    float new_heading = previous_heading - M_PI/2;
    new_heading = correctHead(new_heading);
    return new_heading;
}


//std::cout << curr_node << ". " << mtrn3100::get<0>(coordinates[counter]) << ", " << mtrn3100::get<1>(coordinates[counter]) << ", " << mtrn3100::get<2>(coordinates[counter]) << std::endl;


// // Takes in the shortest path, the length of the path and the initial heading of the robot
// mtrn3100::Tuple<float, float, float> *maze_run (int path[], int size, float start_head) {
  
//     mtrn3100::Tuple<float, float, float> coordinates[100];

//     //initialise all 'prev' variables to match the starting pose
//     float prev_head = start_head;
//     auto firstrc = nodetorowcol(path[0]);
//     int prev_row = mtrn3100::get<0>(firstrc); 
//     int prev_col = mtrn3100::get<1>(firstrc); 
//     int prev_node = path[0];
//     float prev_x = 250*(prev_row) + 125;
//     float prev_y =  250*(prev_col) + 125;
    
//     int counter = 0; //iterates through pose array
//     int path_counter = 0; //iterates through path array
//     while (true) {
//         int curr_node = path[path_counter];
//         auto rc = nodetorowcol(curr_node);
//         int curr_row = mtrn3100::get<0>(rc);
//         int curr_col = mtrn3100::get<1>(rc);
//         float x_pose = prev_x;
//         float y_pose = prev_y;
//         float curr_head = prev_head;
//         float next_head = nodetodirection(prev_node,curr_node,prev_head);

//         if (abs(next_head - curr_head) < 0.01) { //If the current heading matches the direction to be travelled, move cell
//             x_pose = 250*curr_col + 125;
//             y_pose = 250*curr_row + 125;
//             coordinates[counter] = {x_pose, y_pose, curr_head};
//             counter++;
//             path_counter++;

            
//         } else if (abs(abs(next_head - curr_head) - M_PI) < 0.01) { //If the current heading is directly opposite the direction to be travelled, turn right twice
//             curr_node = path[path_counter - 1];
//             curr_head = turnRight(prev_head);
//             coordinates[counter] = {x_pose, y_pose, curr_head};
//             counter++;

//             prev_head = curr_head;
//             curr_head = turnRight(prev_head);
//             coordinates[counter] = {x_pose, y_pose, curr_head};
//             counter++;
//         } else if (abs(abs(next_head - abs(curr_head)) - M_PI/2) < 0.01) { //If the current heading requires one turn to match the direction to be travelled, determine direction to turn
//             curr_node = path[path_counter - 1];
//             if (abs(next_head) - abs(curr_head) < 0.0 || (next_head - M_PI < 0.01 && curr_head + M_PI/2 < 0.01)) { //Turn left
//                     curr_head = turnLeft(prev_head);
//                     coordinates[counter] = {x_pose, y_pose, curr_head};
//                     counter++;
                    
//             } else if (abs(next_head) - abs(curr_head) > 0.0) { //Turn right 
//                 curr_head = turnRight(prev_head);
//                 coordinates[counter] = {x_pose, y_pose, curr_head};
//                 counter++;
//             }
//         }
        

//         prev_row = curr_row;
//         prev_col = curr_col;
//         prev_x = x_pose;
//         prev_y = y_pose;
//         prev_head = curr_head;
//         prev_node = curr_node;
        

//         if (path_counter == size) {
//             break;
//         }
//     }
//     return coordinates;
// };


