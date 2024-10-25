#include <Arduino.h>
#include <math.h>
//const float PI = 3.14159265359;
const int cellWidth = 250;   // in mm
const int cellHeight = 250;  // in mm
const int columns = 9;

struct CellInfo {
  float x_before;
  float y_before;
  float heading_before;
  float x_after;
  float y_after;
  float heading_after;
};

// Function to calculate the center coordinates and heading of a cell
void getCellInfo(int cellNumber, CellInfo &cell, float prevHeading, int prevRow, int prevColumn) {
  int row = (cellNumber - 1) / columns + 1;
  int column = cellNumber - (row - 1) * columns;
  
  // Set the coordinates and heading before the change
  cell.x_before = (prevColumn - 1) * cellWidth;
  cell.y_before = (prevRow - 1) * cellHeight;
  cell.heading_before = prevHeading;
  
  if (fabs(prevHeading - PI / 2) < 0.001) {
    if (prevRow == row) {
      cell.x_after = cell.x_before;
      cell.y_after = cell.y_before;
      cell.heading_after = cell.heading_before;
    } else {
      cell.x_after = cell.x_before;
      cell.y_after = cell.y_before + cellHeight;
      cell.heading_after = cell.heading_before + PI / 2;
    }
  } else if (fabs(prevHeading) < 0.001 || fabs(prevHeading - PI) < 0.001) {
    if (prevColumn == column) {
      cell.x_after = cell.x_before;
      cell.y_after = cell.y_before;
      cell.heading_after = cell.heading_before;
    } else {
      cell.x_after = cell.x_before + cellWidth;
      cell.y_after = cell.y_before;
      cell.heading_after = cell.heading_before + PI / 2;
    }
  } else {
    cell.x_after = cell.x_before;
    cell.y_after = cell.y_before;
    cell.heading_after = cell.heading_before;
  }
}

void setup() {
  Serial.begin(9600);

  // Specify the cell numbers for which you want to get the information
  int cellNumbers[] = {10, 19, 30};  // Example cell numbers
  int numCells = sizeof(cellNumbers) / sizeof(cellNumbers[0]);

  // Initial values for heading, row, and column
  float currentHeading = 0;
  int currentRow = 1;     // Initial row
  int currentColumn = 1;  // Initial column

  // Array to store cell information
  CellInfo cellInfoArray[numCells];

  // Calculate and store the information for each specified cell
  for (int i = 0; i < numCells; i++) {
    getCellInfo(cellNumbers[i], cellInfoArray[i], currentHeading, currentRow, currentColumn);
    currentHeading = cellInfoArray[i].heading_after;  // Update current heading for the next cell
    currentRow = (cellNumbers[i] - 1) / columns + 1;  // Update current row for the next cell
    currentColumn = cellNumbers[i] - (currentRow - 1) * columns; // Update current column for the next cell
  }

  // Print the stored cell information
  for (int i = 0; i < numCells; i++) {
    Serial.print("Cell ");
    Serial.print(cellNumbers[i]);
    Serial.print(" Before Change: (");
    Serial.print(cellInfoArray[i].x_before);
    Serial.print(", ");
    Serial.print(cellInfoArray[i].y_before);
    Serial.print(", ");
    Serial.print(cellInfoArray[i].heading_before);
    Serial.print(") After Change: (");
    Serial.print(cellInfoArray[i].x_after);
    Serial.print(", ");
    Serial.print(cellInfoArray[i].y_after);
    Serial.print(", ");
    Serial.print(cellInfoArray[i].heading_after);
    Serial.println(")");
  }

  while (true) {}  // Uncomment if you want the code to run only once
}

void loop() {
  // Nothing to loop in this example
}
