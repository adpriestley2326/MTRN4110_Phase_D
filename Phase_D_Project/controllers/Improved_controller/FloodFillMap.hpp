// Floodfill-map.hpp
#include <stack>
#include <string>
#include <vector>
#include <queue>

#define MAX_VAL 200

using namespace std;

struct Cell {
  int row;
  int col;
};

enum Direction {
  north,
  east,
  south,
  west,
};

typedef vector<Direction> Path;
string dir2String (Direction d);
Direction turn(Direction d, bool left);
bool oppositeDirections(Direction a, Direction b);
int numTurnsInPath(Path p, Direction initial);
string generateMotionPlan(Path p, Cell initial_cell, Direction initial_dir);
queue<char> generateInstructionQueue(string motionPlan);

class FloodFillMap {
  public:
    Cell target_cell;
    Cell curr_cell;
    Direction curr_dir;
    vector<vector<int>> cell_vals;
    vector<vector<bool>> h_walls;
    vector<vector<bool>> v_walls;
    int n_rows;
    int n_cols;
    
    FloodFillMap();
    FloodFillMap(vector<vector<char>> char_map);
    FloodFillMap(int num_rows, int num_cols, Cell start, Cell target, Direction dir);
    bool validCell(Cell c);
    int getCellValue(Cell c);
    Cell getNeighbourCell(Cell c, Direction dir);
    vector<Cell> getNeighbourCells(Cell c);
    int setCellValue(Cell c, int value);
    bool wallIsPresent(Cell c, Direction dir);
    void addWall(Cell c, Direction dir);
    void doFloodFill(); 
    void updateFloodFill(Cell c);
    vector<Path> findShortestPaths(Cell c);
    Path fewestTurnsPath(Cell c, Direction dir);
    void highlightPath(Path p);
    void display();
};

