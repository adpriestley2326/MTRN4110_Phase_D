#pragma once
// Floodfill-map.hpp
#include <stack>
#include <vector>

#define MAX_VAL 200

using namespace std;

struct Cell {
  int row;
  int col;
};

enum Direction {
  north=0,
  east,
  south,
  west,
};

typedef vector<Direction> Path;

string dir2String (Direction d);

class FloodFillMap {
  public:
    Cell target_cell;
    Cell start_cell;
    Direction start_dir;
    vector<vector<int>> cell_vals;
    vector<vector<bool>> h_walls;
    vector<vector<bool>> v_walls;
    int n_rows;
    int n_cols;
    
    FloodFillMap(vector<vector<char>> char_map);
    FloodFillMap(int num_rows, int num_cols, Cell start, Cell target, Direction dir);
    bool validRowCol(int row, int col);
    int getCellValue(int row, int col);
    Cell getNeighbourCell(int row, int col, Direction dir);
    vector<Cell> getNeighbourCells(Cell c);
    int setCellValue(int row, int col, int value);
    bool wallIsPresent(int row, int col, Direction dir);
    bool addWall(Cell c, Direction dir);
    void doFloodFill(); 
    void updateFloodFill(Cell c);
    vector<Path> findShortestPaths(Cell c);
    void highlightPath(Path p);
    void display();
    void resetValues();
    void setPosition(Cell new_start, Direction dir);
    void changeTarget(Cell new_start, Direction dir, Cell new_target);
    int getAction(Direction d);
};

