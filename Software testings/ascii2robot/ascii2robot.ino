#include <Arduino.h>
#include "Graph.hpp"
#include "ascii2graph.hpp"
#include "LinkedList.hpp"
#include "maze_run.hpp"

// we need to input the maze ascii and the startHead
// currently in serial3.print if want to see output change back to serial.print
// start heading is 0 so if want to move down it will rotate so hence more pose sequence

// instead of hardcoded char maze need to input it through bluetooth
  // Char array to store incoming data

 --- --- --- --- --- --- --- --- --- \n
|       |   |           |           |\n
     ---     --- ---         ---     \n
|   |       |           |       |   |\n
         --- ---             ---     \n
|                   |   |       |   |\n
         --- --- ---     ---     --- \n
|           |                   |   |\n
 ---     --- --- ---     ---         \n
|               |           |       |\n
 --- --- --- --- --- --- --- --- --- \0

#define START_HEAD M_PI / 2  // our start heading
#define CELL_LENGTH 250      // our cell length --> will need to tune
int src = 1;              // starting position
int dst = 10;        // destination position

// Rename tuple to more readable datatype name.
// using MazeCellPose = mtrn3100::Tuple<float, float, float>;

// void generate_and_print_map() {
//   // This is how the nodes are numbered.
//   //  --- --- --- --- --- --- --- --- ---
//   // | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
//   //  --- --- --- --- --- --- --- --- ---
//   // | 10| 11| 12| 13| 14| 15| 16| 17| 18|
//   //  --- --- --- --- --- --- --- --- ---
//   // | 19| 20| 21| 22| 23| 24| 25| 26| 27|
//   //  --- --- --- --- --- --- --- --- ---
//   // | 28| 29| 30| 31| 32| 33| 34| 35| 36|
//   //  --- --- --- --- --- --- --- --- ---
//   // | 37| 38| 39| 40| 41| 42| 43| 44| 45|
//   //  --- --- --- --- --- --- --- --- ---


//   // point: row, col.
  


//   auto const actual = mtrn3100::ascii2graph(maze);
//   // for (auto e : actual.edges()) {
//   //     int src = mtrn3100::get<0>(e.value);
//   //     int dst = mtrn3100::get<1>(e.value);
//   //     Serial.print("{");
//   //     Serial.print(src);
//   //     Serial.print(", ");
//   //     Serial.print(dst);
//   //     Serial.println("},");

//   // }
// }


template<typename N, typename E>
mtrn3100::LinkedList<N> bfs_single(mtrn3100::Graph<N, E> const& g, N const& src, N const& dst) {
  mtrn3100::LinkedList<mtrn3100::LinkedList<N>> queue;
  queue.push_front({ src });

  mtrn3100::LinkedList<N> visited;
  mtrn3100::LinkedList<N> path;

  while (!queue.empty()) {
    mtrn3100::LinkedList<N> currentPath = queue.front();
    queue.pop_front();

    //Getting the current node in the path
    N currentNode = currentPath.back();

    if (currentNode == dst) {
      path = currentPath;
      break;
    }

    // If current node has been visted it skips the for loop
    if (visited.contains(currentNode))
      continue;

    // Adds it the visited path
    visited.push_back(currentNode);

    mtrn3100::LinkedList<N> nodes = g.nodes(currentNode);


    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
      N node = it->value;
      mtrn3100::LinkedList<N> newPath = currentPath;
      newPath.push_back(node);
      queue.push_back(newPath);
    }
  }
  return path;
}


//printing the shortest paths
template<typename N>
void printShortestPaths(const mtrn3100::LinkedList<mtrn3100::LinkedList<N>>& shortestPaths) {
  // Serial3.println(F("Shortest Paths:"));
  for (size_t i = 0; i < shortestPaths.size(); i++) {
    // Serial3.print("Path " + String(i + 1) + ": ");
    for (size_t j = 0; j < shortestPaths[i].size(); j++) {
      Serial3.print(shortestPaths[i][j]);
      if (j < shortestPaths[i].size() - 1) {
        Serial3.print(F(", "));  // change this to an array output
      }
    }
    Serial3.println();
  }
};


template<typename N, typename E>
void addNodesAndEdges(mtrn3100::Graph<N, E>& graph, const int edges[][2], size_t edgeCount) {
  for (size_t i = 0; i < edgeCount; i++) {
    N source = edges[i][0];
    N destination = edges[i][1];
    graph.insert_node(source);
    graph.insert_node(destination);
    graph.insert_edge(source, destination, 1);
  }
};

mtrn3100::Tuple<float, float, float>* yes;

int* maze_run(int path[], int size, float start_head) {
  mtrn3100::Tuple<float, float, float> coordinates[100];

  //initialise all 'prev' variables to match the starting pose
  float prev_head = start_head;
  auto firstrc = nodetorowcol(path[0]);
  int prev_row = mtrn3100::get<0>(firstrc);
  int prev_col = mtrn3100::get<1>(firstrc);
  int prev_node = path[0];
  float prev_x = 250 * (prev_row) + 125;
  float prev_y = 250 * (prev_col) + 125;

  int counter = 0;       //iterates through pose array
  int path_counter = 0;  //iterates through path array
  while (true) {
    int curr_node = path[path_counter];
    auto rc = nodetorowcol(curr_node);
    int curr_row = mtrn3100::get<0>(rc);
    int curr_col = mtrn3100::get<1>(rc);
    float x_pose = prev_x;
    float y_pose = prev_y;
    float curr_head = prev_head;
    float next_head = nodetodirection(prev_node, curr_node, prev_head);

    if (abs(next_head - curr_head) < 0.01) {  //If the current heading matches the direction to be travelled, move cell
      x_pose = 250 * curr_col + 125;
      y_pose = 250 * curr_row + 125;
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;
      path_counter++;


    } else if (abs(abs(next_head - curr_head) - M_PI) < 0.01) {  //If the current heading is directly opposite the direction to be travelled, turn right twice
      curr_node = path[path_counter - 1];
      curr_head = turnRight(prev_head);
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;

      prev_head = curr_head;
      curr_head = turnRight(prev_head);
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;
    } else if (abs(abs(next_head - abs(curr_head)) - M_PI / 2) < 0.01) {  //If the current heading requires one turn to match the direction to be travelled, determine direction to turn
      curr_node = path[path_counter - 1];
      if (abs(next_head) - abs(curr_head) < 0.0 || (next_head - M_PI < 0.01 && curr_head + M_PI / 2 < 0.01)) {  //Turn left
        curr_head = turnLeft(prev_head);
        coordinates[counter] = { x_pose, y_pose, curr_head };
        counter++;

      } else if (abs(next_head) - abs(curr_head) > 0.0) {  //Turn right
        curr_head = turnRight(prev_head);
        coordinates[counter] = { x_pose, y_pose, curr_head };
        counter++;
      }
    }

    prev_row = curr_row;
    prev_col = curr_col;
    prev_x = x_pose;
    prev_y = y_pose;
    prev_head = curr_head;
    prev_node = curr_node;

    if (path_counter == size) {
      break;
    }
  }


  int new_size = 0;
  int new_counter = 0;
  while (true) {
    if (mtrn3100::get<0>(coordinates[new_counter]) == 0.0) {
      break;
    }
    new_counter++;
  }
  new_size = new_counter;
  mtrn3100::Tuple<float, float, float> new_coordinates[new_size];

  for (int i = 0; i < new_size; i++) {
    new_coordinates[i] = coordinates[i];
  }

  yes = new_coordinates;
  return sizeof(new_coordinates) / sizeof(new_coordinates[0]);
};



void setup() {
  Serial.begin(9600);
  //Serial3.begin(9600);

  char maze[429];
  int dataIndex = 0; // Index for the current position in incomingData array 
  int sizeeee=sizeof(" --- --- --- --- --- --- --- --- --- \n|       |   |           |           |\n     ---     --- ---         ---     \n|   |       |           |       |   |\n         --- ---             ---     \n|                   |   |       |   |\n         --- --- ---     ---     --- \n|           |                   |   |\n ---     --- --- ---     ---         \n|               |           |       |\n --- --- --- --- --- --- --- --- --- \0");
  
  
  // while(1){Serial.print(sizeeee);}
  // Wait for data to be available from Serial3 (Bluetooth)
  while (!Serial.available()) {}

  while (dataIndex < 429) {
    if (Serial.available() ) {
      char c = Serial.read();
      if (c=='0'){break;}
      maze[dataIndex] = c; 
      dataIndex++;
      }
      
    }

  // Serial.println(dataIndex-1);
  // Serial.println(sizeof(maze));
  //  Serial.println(maze);
  // Serial.println(maze[-1]);
  //Serial.println(maze[37]);
  char actual[419];
  for(int i=0;i<419;i++){
    actual[i]=maze[i];
  }
  Serial.print(actual);
  while(1){}
  // auto const actual = mtrn3100::ascii2graph(maze);
  //     for (auto e : actual.edges()) {
  //       int src = mtrn3100::get<0>(e.value);
  //       int dst = mtrn3100::get<1>(e.value);
  //       Serial.print(src);
  //       Serial.print(" -- ");
  //       Serial.println(dst);
  //   }
  // mtrn3100::LinkedList<mtrn3100::LinkedList<int>> shortestPaths = bfs_single(actual, src, dst);
  // printShortestPaths(shortestPaths);
  // // Extract the path from shortestPaths
  // mtrn3100::LinkedList<int> shortestPath = shortestPaths[0];  // Assuming you want the first shortest path
  //                                                             // Convert the path to a regular C++ array
  // int pathArray[shortestPath.size()];
  // int index = 0;
  // for (auto it = shortestPath.begin(); it != shortestPath.end(); ++it) {
  //   pathArray[index++] = it->value;
  // }

  // // Call maze_run function with the path array
  // // float START_HEAD = 0; // You might need to set the start heading
  // int mazeSize = maze_run(pathArray, shortestPath.size(), START_HEAD);

  // // Print the coordinates generated by maze_run
  // for (int i = 0; i < mazeSize; i++) {
  //   Serial3.print("{");
  //   Serial3.print(mtrn3100::get<0>(yes[i]));
  //   Serial3.print(", ");
  //   Serial3.print(mtrn3100::get<1>(yes[i]));
  //   Serial3.print(", ");
  //   Serial3.print(mtrn3100::get<2>(yes[i]));
  //   Serial3.println("}");
  // }

}  // end of void setup

void loop() {}