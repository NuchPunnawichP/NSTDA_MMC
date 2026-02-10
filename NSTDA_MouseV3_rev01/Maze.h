#include <Arduino.h>

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

const int neighboringCells[4][2] = {
  { -1, 0 },
  { 0, 1 },
  { 1, 0 },
  { 0, -1 },
};

const byte neighboringWalls[4][2] = {
  { 0, 0 },
  { 0, 1 },
  { 1, 0 },
  { 0, 0 }
};

template<byte ROWS, byte COLUMNS>
class MouseMaze {
private:
  // vertical walls array
  boolean verticalWalls[ROWS][COLUMNS + 1];

  // horizontal walls array
  boolean horizontalWalls[ROWS + 1][COLUMNS];

public:
  // value array
  byte values[ROWS][COLUMNS];

  byte mouseRow;      // Posición inicial del Mouse en la Fila
  byte mouseColumn;   // Posición inicial del Mouse en la Columna
  byte mouseHeading;  // Dirección inicial del Mouse

  byte targetRow;
  byte targetColumn;

  //Constructor method (called when the maze is created)
  MouseMaze() {
    //initialize verticalWalls (add exterior walls)
    for (byte i = 0; i < ROWS; i++) {
      for (byte j = 0; j < COLUMNS + 1; j++) {
        if (j == 0 || j == COLUMNS) {
          verticalWalls[i][j] = true;
        }
      }
    }

    //initialize horizontalWalls (add exterior walls)
    for (byte i = 0; i < ROWS + 1; i++) {
      for (byte j = 0; j < COLUMNS; j++) {
        if (i == 0 || i == ROWS) {
          horizontalWalls[i][j] = true;
        }
      }
    }
  }

  //Add your code here
  void solveFloodFill() {

    // Initialize Array
    for (byte i = 0; i < ROWS; i++) {
      for (byte j = 0; j < COLUMNS; j++) {
        values[i][j] = 255;
      }
    }

    // Set target cell
    values[targetRow][targetColumn] = 0;

    // The Solving
    bool continueSolving = true;

    while (continueSolving) {

      continueSolving = false;

      for (byte i = 0; i < ROWS; i++) {
        for (byte j = 0; j < COLUMNS; j++) {
          if (values[i][j] < 255) {
            for (byte k = 0; k < 4; k++) {
              int neighboringCellRow = i + neighboringCells[k][0];
              int neighboringCellColumn = j + neighboringCells[k][1];

              byte neighboringWallRow = i + neighboringWalls[k][0];
              byte neighboringWallColumn = j + neighboringWalls[k][1];

              bool wallExists = false;

              if (k == NORTH || k == SOUTH)
                wallExists = horizontalWalls[neighboringWallRow][neighboringWallColumn];
              else  // Must be loking at an EAST or WEST wall
                wallExists = verticalWalls[neighboringWallRow][neighboringWallColumn];
              //Serial.print("i:"); Serial.print(i); Serial.print(" - j:"); Serial.print(j); Serial.print(" - k:"); Serial.print(k); Serial.print(" --- "); Serial.println(wallExists);
              if (values[neighboringCellRow][neighboringCellColumn] == 255 && !wallExists) {
                values[neighboringCellRow][neighboringCellColumn] = values[i][j] + 1;
                continueSolving = true;  // If update, then continueSolving
              }
            }
          }
        }
      }
    }
  }

  byte findNextMove() {

    byte valueBestNeighbor = 255;
    byte desiredHeading = 0;

    for (byte k = 0; k < 4; k++) {

      // Coordenada [Row,Column] de la siguiente Celda en esta direccion
      int neighboringCellRow = mouseRow + neighboringCells[k][0];
      int neighboringCellColumn = mouseColumn + neighboringCells[k][1];

      // Coordenada [Row,Column] del siguiente espacio para una 'posible' pared en esta dirección
      byte neighboringWallRow = mouseRow + neighboringWalls[k][0];
      byte neighboringWallColumn = mouseColumn + neighboringWalls[k][1];

      // Verificar si existe pared en esta direccion
      bool wallExists = false;
      if (k == NORTH || k == SOUTH)
        wallExists = horizontalWalls[neighboringWallRow][neighboringWallColumn];
      else  // Must be loking at an EAST or WEST wall
        wallExists = verticalWalls[neighboringWallRow][neighboringWallColumn];

      //Serial.print("i:"); Serial.print(mouseRow); Serial.print(" - j:"); Serial.print(mouseColumn); Serial.print(" - k:"); Serial.print(k); Serial.print(" --- "); Serial.println(wallExists);

      // Si la siguiente celda en esta direccion tiene un Valor menor a las otras direcciones
      if (values[neighboringCellRow][neighboringCellColumn] < valueBestNeighbor && !wallExists
          || values[neighboringCellRow][neighboringCellColumn] == valueBestNeighbor && !wallExists && k == mouseHeading) {  // Para favorecer a mouseHeading si tiene el valor menor como otro (Probar mouseRow:3, mouseColumn:3, mouseHeading:WEST)
        valueBestNeighbor = values[neighboringCellRow][neighboringCellColumn];                                              // Si asi, esta siguiente Celda es la mejor para continuar y se indica la dereccion deseada
        desiredHeading = k;
      }
    }

    return desiredHeading;
  }

  void addWalls(byte cardinalDirection) {

    switch (cardinalDirection) {
      case NORTH:
        horizontalWalls[mouseRow][mouseColumn] = true;
        break;
      case EAST:
        verticalWalls[mouseRow][mouseColumn + 1] = true;
        break;
      case SOUTH:
        horizontalWalls[mouseRow + 1][mouseColumn] = true;
        break;
      case WEST:
        verticalWalls[mouseRow][mouseColumn] = true;
        break;
    }
  }

  void addVirtualWalls_test2(){
    verticalWalls[1][1] = true;
    verticalWalls[2][1] = true;
    verticalWalls[4][1] = true;
    verticalWalls[5][1] = true;
    verticalWalls[6][1] = true;
    verticalWalls[7][1] = true;
    verticalWalls[8][1] = true;
    verticalWalls[10][1] = true;

    verticalWalls[0][2] = true;
    verticalWalls[2][2] = true;
    verticalWalls[5][2] = true;
    verticalWalls[6][2] = true;
    verticalWalls[7][2] = true;
    verticalWalls[9][2] = true;

    verticalWalls[1][3] = true;
    verticalWalls[3][3] = true;
    verticalWalls[5][3] = true;
    verticalWalls[6][3] = true;
    verticalWalls[8][3] = true;

    verticalWalls[0][4] = true;
    verticalWalls[2][4] = true;
    verticalWalls[4][4] = true;
    verticalWalls[7][4] = true;

    verticalWalls[1][5] = true;
    verticalWalls[4][5] = true;
    verticalWalls[6][5] = true;
    
    verticalWalls[0][6] = true;
    verticalWalls[2][6] = true;
    verticalWalls[3][6] = true;
    verticalWalls[5][6] = true;
    verticalWalls[8][6] = true;

    verticalWalls[1][7] = true;

    verticalWalls[0][8] = true;
    verticalWalls[2][8] = true;
    verticalWalls[5][8] = true;

    verticalWalls[1][9] = true;
    verticalWalls[5][9] = true;

    verticalWalls[0][10] = true;
    verticalWalls[9][10] = true;

    horizontalWalls[2][6] = true;
    horizontalWalls[2][7] = true;
    horizontalWalls[2][9] = true;

    horizontalWalls[3][3] = true;
    horizontalWalls[3][4] = true;
    horizontalWalls[3][9] = true;

    horizontalWalls[4][2] = true;
    horizontalWalls[4][6] = true;
    horizontalWalls[4][7] = true;
    horizontalWalls[4][9] = true;

    horizontalWalls[5][3] = true;
    horizontalWalls[5][4] = true;
    horizontalWalls[5][7] = true;
    horizontalWalls[5][9] = true;

    horizontalWalls[6][2] = true;
    horizontalWalls[6][3] = true;
    horizontalWalls[6][5] = true;
    horizontalWalls[6][7] = true;
    horizontalWalls[6][9] = true;

    horizontalWalls[7][6] = true;
    horizontalWalls[7][7] = true;
    horizontalWalls[7][8] = true;
    horizontalWalls[7][9] = true;
    horizontalWalls[7][10] = true;

    horizontalWalls[8][3] = true;
    horizontalWalls[8][5] = true;
    horizontalWalls[8][6] = true;
    horizontalWalls[8][7] = true;
    horizontalWalls[8][9] = true;

    horizontalWalls[9][2] = true;
    horizontalWalls[9][4] = true;
    horizontalWalls[9][5] = true;
    horizontalWalls[9][6] = true;
    horizontalWalls[9][7] = true;
    horizontalWalls[9][8] = true;

    horizontalWalls[10][3] = true;
    horizontalWalls[10][4] = true;
    horizontalWalls[10][5] = true;
    horizontalWalls[10][6] = true;
    horizontalWalls[10][7] = true;

  }

  void addVirtualWalls() {
    verticalWalls[1][1] = true;
    verticalWalls[2][1] = true;
    verticalWalls[4][1] = true;
    verticalWalls[6][1] = true;
    verticalWalls[7][1] = true;
    verticalWalls[8][1] = true;
    verticalWalls[9][1] = true;
    verticalWalls[10][1] = true;
    verticalWalls[11][1] = true;
    verticalWalls[12][1] = true;
    verticalWalls[13][1] = true;
    verticalWalls[15][1] = true;

    verticalWalls[1][2] = true;
    verticalWalls[2][2] = true;
    verticalWalls[4][2] = true;
    verticalWalls[7][2] = true;
    verticalWalls[8][2] = true;
    verticalWalls[11][2] = true;
    verticalWalls[13][2] = true;

    verticalWalls[1][3] = true;
    verticalWalls[2][3] = true;
    verticalWalls[7][3] = true;
    verticalWalls[12][3] = true;

    verticalWalls[1][4] = true;
    verticalWalls[11][4] = true;

    verticalWalls[6][5] = true;

    verticalWalls[2][6] = true;
    verticalWalls[6][6] = true;
    verticalWalls[9][6] = true;
    verticalWalls[14][6] = true;

    verticalWalls[2][7] = true;
    verticalWalls[4][7] = true;
    verticalWalls[7][7] = true;
    verticalWalls[8][7] = true;
    verticalWalls[13][7] = true;

    verticalWalls[1][8] = true;
    verticalWalls[2][8] = true;
    verticalWalls[5][8] = true;
    verticalWalls[12][8] = true;
    verticalWalls[14][8] = true;

    verticalWalls[2][9] = true;
    verticalWalls[4][9] = true;
    verticalWalls[6][9] = true;
    verticalWalls[7][9] = true;
    verticalWalls[9][9] = true;
    verticalWalls[10][9] = true;
    verticalWalls[11][9] = true;
    verticalWalls[14][9] = true;
    verticalWalls[15][9] = true;

    verticalWalls[1][10] = true;
    verticalWalls[2][10] = true;
    verticalWalls[5][10] = true;
    verticalWalls[6][10] = true;
    verticalWalls[7][10] = true;
    verticalWalls[9][10] = true;
    verticalWalls[10][10] = true;
    verticalWalls[12][10] = true;
    verticalWalls[14][10] = true;

    verticalWalls[5][11] = true;
    verticalWalls[6][11] = true;
    verticalWalls[7][11] = true;
    verticalWalls[9][11] = true;
    verticalWalls[10][11] = true;
    verticalWalls[11][11] = true;
    verticalWalls[13][11] = true;

    verticalWalls[1][12] = true;
    verticalWalls[2][12] = true;
    verticalWalls[4][12] = true;
    verticalWalls[5][12] = true;
    verticalWalls[6][12] = true;
    verticalWalls[7][12] = true;
    verticalWalls[9][12] = true;
    verticalWalls[10][12] = true;
    verticalWalls[11][12] = true;
    verticalWalls[12][12] = true;
    verticalWalls[14][12] = true;

    verticalWalls[1][13] = true;
    verticalWalls[2][13] = true;
    verticalWalls[5][13] = true;
    verticalWalls[6][13] = true;
    verticalWalls[7][13] = true;
    verticalWalls[9][13] = true;
    verticalWalls[10][13] = true;
    verticalWalls[11][13] = true;
    verticalWalls[12][13] = true;
    verticalWalls[13][13] = true;
    verticalWalls[15][13] = true;

    verticalWalls[3][14] = true;
    verticalWalls[6][14] = true;
    verticalWalls[7][14] = true;
    verticalWalls[9][14] = true;
    verticalWalls[10][14] = true;
    verticalWalls[11][14] = true;
    verticalWalls[12][14] = true;
    verticalWalls[14][14] = true;

    verticalWalls[1][15] = true;
    verticalWalls[2][15] = true;
    verticalWalls[5][15] = true;
    verticalWalls[7][15] = true;
    verticalWalls[9][15] = true;
    verticalWalls[10][15] = true;
    verticalWalls[11][15] = true;
    verticalWalls[13][15] = true;
    verticalWalls[14][15] = true;

    horizontalWalls[1][1] = true;


    // verticalWalls[1][1] = true;
    // verticalWalls[1][2] = true;
    // horizontalWalls[1][1] = true;
    // verticalWalls[0][1] = true;
    // verticalWalls[1][1] = true;
    // verticalWalls[3][1] = true;

    // verticalWalls[1][2] = true;
    // verticalWalls[2][2] = true;

    // verticalWalls[2][3] = true;

    // verticalWalls[2][4] = true;

    // verticalWalls[2][5] = true;

    // horizontalWalls[1][2] = true;
    // horizontalWalls[1][3] = true;
    // horizontalWalls[1][4] = true;

    // horizontalWalls[2][3] = true;

    // horizontalWalls[3][4] = true;

    //verticalWalls[1][3] = true;
  }

  /*Do not change or add code below this line
    
    NanoMouseMaze Print Function Version 2
    This version of the print function has been modified to print
    any size maze (the previous version could not print large
    mazes) and to work with the btMonitor Android App I wrote,
    Scroll down to "Wireless Debugging with the Bluetooth Module"
    and go to the Downloadable Materials section of the lecture.*/

  void printMaze() {
    for (byte i = 0; i < 2 * ROWS + 1; i++) {
      for (byte j = 0; j < 2 * COLUMNS + 1; j++) {
        //Add Horizontal Walls
        if (i % 2 == 0 && j % 2 == 1) {
          if (horizontalWalls[i / 2][j / 2] == true) {
            Serial.print(" ---");
          } else {
            Serial.print("    ");
          }
        }

        //Add Vertical Walls
        if (i % 2 == 1 && j % 2 == 0) {
          if (verticalWalls[i / 2][j / 2] == true) {
            Serial.print("|");
          } else {
            Serial.print(" ");
          }
        }

        //Add Flood Fill Values
        if (i % 2 == 1 && j % 2 == 1) {
          if ((i - 1) / 2 == mouseRow && (j - 1) / 2 == mouseColumn) {
            if (mouseHeading == NORTH) {
              Serial.print(" ^ ");
            } else if (mouseHeading == EAST) {
              Serial.print(" > ");
            } else if (mouseHeading == SOUTH) {
              Serial.print(" v ");
            } else if (mouseHeading == WEST) {
              Serial.print(" < ");
            }
          } else {
            byte value = values[(i - 1) / 2][(j - 1) / 2];
            if (value >= 100) {
              Serial.print(value);
            } else {
              Serial.print(" ");
              Serial.print(value);
            }
            if (value < 10) {
              Serial.print(" ");
            }
          }
        }
      }
      Serial.print("\n");
    }
    Serial.print("\n");
    
  }

};