#include <Arduino.h>
#include "Graph.hpp"
#include "ascii2graph.hpp"
#include "LinkedList.hpp"
#include "maze_run.hpp"

// we need to input the maze ascii and the startHead
// currently in Serial.print if want to see output change back to serial.print
// start heading is 0 so if want to move down it will rotate so hence more pose sequence
#define SOURCE_CELL 1
#define DST_CELL 10

char maze[] =
  " --- --- --- --- --- --- --- --- --- \n"
  "|               |       |           |\n"
  " --- ---     --- ---     ---     --- \n"
  "|                               |   |\n"
  " --- --- --- --- ---                 \n"
  "|       |           |               |\n"
  "     ---         --- ---     --- --- \n"
  "|                           |       |\n"
  "             --- --- ---     ---     \n"
  "|                       |       |   |\n"
  " --- --- --- --- --- --- --- --- --- \0";

float startHead = 0;  // our startinging heading


// Helper function.
void insert_edge(mtrn3100::Graph<int, int>& maze, int cell1, int cell2) {
  // Graph is a directed graph so need to do both directions.
  maze.insert_edge(cell1, cell2, 0);
  maze.insert_edge(cell2, cell1, 0);
}

void generate_and_print_map() {

  auto const actual = mtrn3100::ascii2graph(maze);
  for (auto e : actual.edges()) {
    int src = mtrn3100::get<0>(e.value);
    int dst = mtrn3100::get<1>(e.value);
    Serial.print("{");
    Serial.print(src);
    Serial.print(", ");
    Serial.print(dst);
    Serial.println("},");
  }
}


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
  // Serial.println(F("Shortest Paths:"));
  for (size_t i = 0; i < shortestPaths.size(); i++) {
    // Serial.print("Path " + String(i + 1) + ": ");
    for (size_t j = 0; j < shortestPaths[i].size(); j++) {
      Serial.print(shortestPaths[i][j]);
      if (j < shortestPaths[i].size() - 1) {
        Serial.print(F(", "));  // change this to an array output
      }
    }
    Serial.println();
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

mtrn3100::Tuple<float, float, float>* poseSequence;

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

  poseSequence = new_coordinates;
  return sizeof(new_coordinates) / sizeof(new_coordinates[0]);
};
void ascii2pose(){
  int src = SOURCE_CELL;        // starting position
  int dst = DST_CELL;  // destination position
  // int ascii_size=sizeof(maze);
  // while(1){Serial.println(ascii_size);}
  auto const actual = mtrn3100::ascii2graph(maze);
  mtrn3100::LinkedList<mtrn3100::LinkedList<int>> shortestPaths = bfs_single(actual, src, dst);
  printShortestPaths(shortestPaths);
  // Extract the path from shortestPaths
  mtrn3100::LinkedList<int> shortestPath = shortestPaths[0];  // Assuming you want the first shortest path
                                                              // Convert the path to a regular C++ array
  int pathArray[shortestPath.size()];
  int index = 0;
  for (auto it = shortestPath.begin(); it != shortestPath.end(); ++it) {
    pathArray[index++] = it->value;
  }

  // Call maze_run function with the path array
  // float startHead = 0; // You might need to set the start heading
  int mazeSize = maze_run(pathArray, shortestPath.size(), startHead);

  // Print the coordinates generated by maze_run
  for (int i = 0; i < mazeSize; i++) {
    Serial.print("{");
    Serial.print(mtrn3100::get<0>(poseSequence[i]));
    Serial.print(", ");
    Serial.print(mtrn3100::get<1>(poseSequence[i]));
    Serial.print(", ");
    Serial.print(mtrn3100::get<2>(poseSequence[i]));
    Serial.println("}");
  }
}

void setup() {

  ascii2pose();

}  // end of void setup

void loop() {
}