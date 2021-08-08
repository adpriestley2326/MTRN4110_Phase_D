// File:          z5207265_MTRN4110_PhaseB.cpp
// Date:          06/07/2021
// Description:   Controller of E-puck for Phase B - Path Planning
// Author:        Adam Priestley
// Modifications:
// Platform:      Windows
// Notes:         

#include <webots/Robot.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define N 200

using namespace webots;
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

const string MAP_FILE_NAME = "../../MapBuilt.txt";
const string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const string OUTPUT_FILE_NAME = "../../Output.txt";
fstream output_fs;
fstream path_fs;
vector<Direction> all_directions = {north,east,south,west};

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
    bool validRowCol(int row, int col);
    int getCellValue(int row, int col);
    Cell getNeighbourCell (int row, int col, Direction dir);
    int setCellValue(int row, int col, int value);
    bool wallIsPresent(int row, int col, Direction dir);
    void doFloodFill(); 
    vector<Path> findShortestPaths(Cell c);
    void highlightPath(Path p);
    void display();
};

void myPrint(string s);
vector<char> store_row_string(string s);
string dir2String (Direction d);
string cell2String (Cell c);
int numTurnsInPath(Path p, Direction initial);
string generateMotionPlan(Path p, Cell initial_cell, Direction initial_dir);


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  // Open files for writing
  output_fs.open(OUTPUT_FILE_NAME, ios::out);
  path_fs.open(PATH_PLAN_FILE_NAME, ios::out);

  // Task 1a - Open map file
  myPrint("Reading in map from " + MAP_FILE_NAME + "...");
  fstream map_fs(MAP_FILE_NAME, ios::in);
  if (!map_fs.is_open()) {
    myPrint("Could not read map file!");
    return 1;
  }
  
  // Task 1b - Read map file
  string map_line;
  vector<vector<char>> char_map;
  while (getline(map_fs, map_line)) {
    myPrint(map_line);
    vector<char> row = store_row_string(map_line);
    char_map.push_back(row);
  }
  map_fs.close();
  myPrint("Map read in!");
  
  
  // Task 2a - Do floodfill
  FloodFillMap map(char_map);
  map.doFloodFill();
  
  // Task 2b - Find & display shortest paths
  myPrint("Finding shortest paths...");
  auto paths = map.findShortestPaths(map.start_cell);
  int pNum = 0;
  for (auto path_it = paths.begin(); path_it != paths.end(); path_it++) {
    myPrint("Path - " + to_string(++pNum) + ":");
    map.highlightPath(*path_it);
    map.display();
  }
  myPrint(to_string(paths.size()) + " shortest paths found!");  
  
  
  // Task 3a - Check for fewest turns
  myPrint("Finding shortest path with least turns...");
  Path fewest_turns_path = paths[0];
  for (auto path_it = paths.begin(); path_it != paths.end(); path_it++) {
    if (numTurnsInPath(*path_it, map.start_dir) <
        numTurnsInPath(fewest_turns_path, map.start_dir))
    {
      fewest_turns_path = *path_it;
    }
  }
  map.highlightPath(fewest_turns_path);
  map.display();
  myPrint("Shortest path with least turns found!");
  
  // Task 3b - Generate motion plan
  string motion_plan = generateMotionPlan(fewest_turns_path, map.start_cell, map.start_dir);
  int motion_plan_length = motion_plan.length() - 3;
  myPrint("Path Plan (" + to_string(motion_plan_length) + " steps): " + motion_plan);
  
  
  // Task 4 - Write path to text file
  myPrint("Writing path plan to " + PATH_PLAN_FILE_NAME + "...");
  if (!path_fs.is_open()) {
    myPrint("Error - could not write to path file!");
    return -1;
  }
  path_fs << motion_plan;
  path_fs.close();
  myPrint("Path plan written to " + PATH_PLAN_FILE_NAME + "!");
  
  // Cleanup
  delete robot;
  output_fs.close();

  return 0;
}




// ------------ Helper functions ---------------

void myPrint(string s) {
  string out_s = "[z5207265_MTRN4110_PhaseB] " + s + "\n";
  
  cout << out_s; // Write to console
  if (output_fs.is_open()) { // Check output filestream
    output_fs << out_s; // Write to output file
  } else {
    cout << "Warning - could not write to output file!\n";
  }
  return;
}

vector<char> store_row_string (string s) {
  vector<char> row;
  for (int i = 0; i < (int) s.length(); i++) {
    if (i % 2 != 0) continue;
    row.push_back((char) s[i]);
  }
  return row;
}

string dir2String (Direction d) {
  switch (d) {
    case north: return "^";
    case east: return ">";
    case south: return "v";
    case west: return "<";
  }
  return "o";
}

string cell2String (Cell c) {
  return "(" + to_string(c.row) + ", " + to_string(c.col) + ")";
}

bool oppositeDirections(Direction a, Direction b) {
  switch (a) {
    case north: return b == south;
    case south: return b == north;
    case east: return b == west;
    case west: return b == east;
  }
  return false;
}

int numTurnsInPath(Path p, Direction initial) {
  int turn_count = 0;
  Direction facing = initial;
  for (auto dir_it = p.rbegin(); dir_it != p.rend(); dir_it++) {
    if (oppositeDirections(facing, *dir_it)) turn_count += 2;
    else if (facing != *dir_it) turn_count += 1;
    facing = *dir_it;
  }
  return turn_count;
}

string generateMotionPlan(Path p, Cell initial_cell, Direction initial_dir) {
  string plan = "";
  plan.append(to_string(initial_cell.row - 1) + to_string(initial_cell.col - 1));
  switch (initial_dir) {
    case north:
      plan.append("N");
      break;
    case south:
      plan.append("S");
      break;
    case east:
      plan.append("E");
      break;
    case west:
      plan.append("W");
      break;
    default:
      plan.append("-");
  }
  Direction facing = initial_dir;
  auto dir_it = p.rbegin();
  while(dir_it != p.rend()) {
    if (facing == *dir_it) {
      plan.append("F");
      dir_it++;
    } else if (oppositeDirections(facing, *dir_it)) {
      plan.append("LL");
      facing = *dir_it;
    } else if (facing != *dir_it) {
      switch (*dir_it) {
        case north:
          plan.append((facing == east) ? "L" : "R");
          break;
        case south:
          plan.append((facing == west) ? "L" : "R");
          break;
        case east:
          plan.append((facing == south) ? "L" : "R");
          break;
        case west:
          plan.append((facing == north) ? "L" : "R");
          break;
      }
      facing = *dir_it;
    }
  }  
  return plan;
}


// ------- FloodFillMap member functions --------

FloodFillMap::FloodFillMap (vector<vector<char>> char_map) {
  // myPrint("Creating floodfill map...");
  n_rows = (int) char_map.size() / 2;
  n_cols = (int) char_map[0].size() / 2;
  for (int r = 0; r < (int)char_map.size(); r++) {
    vector<bool> walls_row_vec;
    vector<int> row_vec;
    for (int c = 0; c < (int)char_map[r].size(); c++) {
      if (r % 2 == 0) { 
        if (c % 2 != 0) { // Horizontal wall or lack thereof
          //myPrint("Checking horizontal wall at (" + to_string(r) + ", " + to_string(c) + ")");
          walls_row_vec.push_back(char_map[r][c] == '-');
          //myPrint("    Wall stored.");
        }
      } else if (c % 2 == 0) { 
        if (r % 2 != 0) { // Vertical wall or lack thereof
          //myPrint("Checking vertical wall at (" + to_string(r) + ", " + to_string(c) + ")");
          walls_row_vec.push_back(char_map[r][c] == '|');
        }
      } else { // Cell - not wall
        int cell_val = N;
        if (char_map[r][c] == 'x') { // target cell found
          target_cell = {(r+1)/2, (c+1)/2}; // record its position
          cell_val = 0;
        } else if (char_map[r][c] != ' ') { // directional cell
          start_cell = {(r+1)/2, (c+1)/2};
          switch (char_map[r][c]) {
            case '^':
              start_dir = north;
              break;
            case '>':
              start_dir = east;
              break;
            case '<':
              start_dir = west;
              break;
            case 'v':
              start_dir = south;
              break;
          }
        }
        row_vec.push_back(cell_val);
      }
    }
    if (r % 2 == 0) {
      h_walls.push_back(walls_row_vec);
    } else {
      v_walls.push_back(walls_row_vec);
      cell_vals.push_back(row_vec);
    }
    // myPrint("    Row " + to_string(r) + " complete!");
  }      
}

bool FloodFillMap::validRowCol (int row, int col) {
  if (row < 1 || row > n_rows) return false;
  if (col < 1 || col > n_cols) return false;
  return true;
}
  
int FloodFillMap::getCellValue (int row, int col) {
  if (!validRowCol(row,col)) return -1;
  return cell_vals[row - 1][col - 1];
}

Cell FloodFillMap::getNeighbourCell (int row, int col, Direction dir) {
  Cell neighbour;
  switch (dir) {
    case north:
      neighbour = {row - 1, col}; break;
    case east:
      neighbour = {row, col + 1}; break;
    case south:
      neighbour = {row + 1, col}; break;
    case west:
      neighbour = {row, col - 1}; break;
  }
  return neighbour;
}

int FloodFillMap::setCellValue (int row, int col, int value) {
  if (!validRowCol(row,col)) return -1;
  cell_vals[row - 1][col - 1] = value;
  return value;
}

bool FloodFillMap::wallIsPresent (int row, int col, Direction dir) {
  if (!validRowCol(row,col)) {
    myPrint("Invalid row or col for wallIsPresent!");
    return false;
  }
  switch (dir) {
    case north: return h_walls[row-1][col-1];
    case south: return h_walls[row][col-1];
    case east:  return v_walls[row-1][col];
    case west:  return v_walls[row-1][col-1];
    default:
      myPrint("Invalid direction passed to wallIsPresent!");
      return false;
  }
}

void FloodFillMap::doFloodFill() {
  int curr_explored_val = 0;
  bool maze_val_changed = true;
  while (maze_val_changed) {
    maze_val_changed = false;
    for (int r = 1; r <= n_rows; r++) {
      for (int c = 1; c <= n_cols; c++) {
        int curr_cell_val = getCellValue(r,c);
        if (curr_cell_val == curr_explored_val) {
          //cout << "\n\nr" << to_string(r) << " c" << to_string(c) << ":\n";
          for (auto direction_it = all_directions.begin();
                      direction_it != all_directions.end();
                      direction_it++)
            {
            //cout << "  " << dir2String(*direction_it) << ": ";
            if (!wallIsPresent(r,c,*direction_it)) {
              //cout << "open - ";
              Cell neighbour = getNeighbourCell(r,c,*direction_it);
              if (getCellValue(neighbour.row, neighbour.col) == N) {
                //cout << "Changing value of cell: " << cell2String(neighbour)
                //     << " to " << curr_cell_val+1 << '\n';
                setCellValue(neighbour.row, neighbour.col, curr_cell_val + 1);
                maze_val_changed = true;
              }// else cout << "Already explored\n";
            }// else cout << "walled\n";
          }
        }
      }
    }
    curr_explored_val++;
    //cout << "\n \n---- Now looking at cells w. value " << curr_explored_val << "-------";
  }
  return;
}

vector<Path> FloodFillMap::findShortestPaths(Cell c) {
  vector<Path> paths;
  int curr_val = getCellValue(c.row, c.col);
  if (curr_val == 0) {
    Path empty_path;
    paths.push_back(empty_path);
    return paths; // Already at destination
  }
  for (auto dir_it = all_directions.begin(); dir_it != all_directions.end(); dir_it++) {
    if (wallIsPresent(c.row,c.col,*dir_it)) continue;
    Cell neighbour = getNeighbourCell(c.row,c.col,*dir_it);
    if (getCellValue(neighbour.row,neighbour.col) == curr_val - 1) {
      auto sub_paths = findShortestPaths(neighbour);
      for (auto path = sub_paths.begin(); path != sub_paths.end(); path++) {
        (*path).push_back(*dir_it);
        paths.push_back(*path);
      }
    }
  }
  return paths;
}

void FloodFillMap::highlightPath(Path p) {
  int index = p.size();
  Cell curr = start_cell;
  for (int r = 1; r <= n_rows; r++) {
    for (int c = 1; c <= n_cols; c++) {
      setCellValue(r, c, N); // Empty all cell values
    }
  }
  for (auto dir_it = p.rbegin(); dir_it != p.rend(); dir_it++) {
    Cell next = getNeighbourCell(curr.row, curr.col, *dir_it);
    setCellValue(next.row, next.col, --index); // Refill values on the path
    curr = next;
  }
}

void FloodFillMap::display() {
  bool h_wall = true;
  bool v_wall = true;
  auto h_walls_row = h_walls.begin();
  auto v_walls_row = v_walls.begin();
  Cell curr = {1,1};
  for (int r = 0; r < (int)(cell_vals.size() + h_walls.size()); r++) {
    string row_string = "";
    v_wall = true;
    if (h_wall) { // Row with horizontal walls
      auto h_walls_it = h_walls_row->begin();
      while (h_walls_it != h_walls_row->end()) {
        if (v_wall) row_string.append(" ");
        else row_string.append(*(h_walls_it++) ? "---" : "   ");
        v_wall = !v_wall;
      }
      row_string.append(" ");
    } else { // Row with cells or vertical walls
      auto v_walls_it = v_walls_row->begin();
      curr.col = 1;
      while (v_walls_it != v_walls_row->end()) {
        if (v_wall) row_string.append(*(v_walls_it++) ? "|" : " "); // vertical wall col
        else { // cell col
          if (curr.row == start_cell.row && curr.col == start_cell.col) {
            // Display starting position
            row_string.append(" " + dir2String(start_dir) + " ");
          /*} else if (curr.row == target_cell.row && curr.col == target_cell.col) {
            // Display target position
            row_string.append(" x ");*/
          } else if (getCellValue(curr.row, curr.col) == N) {
            // Display empty cell
            row_string.append("   ");
          } else {
            // Display path index of cell
            string value = to_string(getCellValue(curr.row, curr.col));
            string rear_padding = string(2 - value.length(), ' ');
            row_string.append(" " + value + rear_padding);
          }
          curr.col++;
        }
        v_wall = !v_wall;
      }
      curr.row++;
      h_walls_row++;
      v_walls_row++;
    }
    h_wall = !h_wall;
    myPrint(row_string);
  }
}
