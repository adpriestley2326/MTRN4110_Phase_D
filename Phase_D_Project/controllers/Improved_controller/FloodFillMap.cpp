// FloodFillMap.cpp

#include <iostream>
#include <string>
#include <FloodFillMap.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

string dir2String (Direction d) {
  switch (d) {
    case north: return "^";
    case east: return ">";
    case south: return "v";
    case west: return "<";
  }
  return "o";
}

Direction turn(Direction d, bool left) {
  switch (d) {
    case north: return left ? west : east;
    case east: return left ? north : south;
    case south: return left ? east : west;
    case west: return left ? south : north;
  }
  return north; // should never happen
}

const string OUTPUT_FILE_NAME = "../../Output.txt";

vector<Direction> all_directions = {north,east,south,west};

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
  if (p.empty()) return plan;
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

queue<char> generateInstructionQueue(string motionPlan) {
  queue<char> instructions;
  if (motionPlan.size() == 3) {
    return instructions;
  }
  motionPlan.erase(0,3);
  for (auto char_it = motionPlan.begin(); char_it != motionPlan.end(); char_it++) {
    instructions.push(*char_it);
  }
  return instructions;
}

// ----------- FloodFillMap Class -----------

FloodFillMap::FloodFillMap () {
  n_rows = 5;
  n_cols = 9;
  curr_cell = {1,1};
  curr_dir = south;
  target_cell = {1,1};
  
  // Initialise empty maze 
  // Set up horizontal walls
  h_walls = vector<vector<bool>>(n_rows+1, vector<bool>(n_cols, false));
  h_walls[0] = vector<bool>(n_cols, true);
  h_walls[n_rows] = vector<bool>(n_cols, true);
  
  // Set up vertical walls
  vector<bool> v_walls_row(n_cols+1, false);
  v_walls_row[0] = true;
  v_walls_row[n_cols] = true;
  v_walls = vector<vector<bool>>(n_rows, v_walls_row);
  
  // Set up cell values (distances from target)
  cell_vals = vector<vector<int>>(n_rows, vector<int>(n_cols, MAX_VAL));
  cell_vals[target_cell.row-1][target_cell.col-1] = 0;
}


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
          walls_row_vec.push_back(char_map[r][c] == '-');
        }
      } else if (c % 2 == 0) { 
        if (r % 2 != 0) { // Vertical wall or lack thereof
          walls_row_vec.push_back(char_map[r][c] == '|');
        }
      } else { // Cell - not wall
        int cell_val = MAX_VAL;
        if (char_map[r][c] == 'x') { // target cell found
          target_cell = {(r+1)/2, (c+1)/2}; // record its position
          cell_val = 0;
        } else if (char_map[r][c] != ' ') { // directional cell
          curr_cell = {(r+1)/2, (c+1)/2};
          switch (char_map[r][c]) {
            case '^':
              curr_dir = north;
              break;
            case '>':
              curr_dir = east;
              break;
            case '<':
              curr_dir = west;
              break;
            case 'v':
              curr_dir = south;
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

FloodFillMap::FloodFillMap (int num_rows, int num_cols, Cell start, Cell target, Direction dir) {
  n_rows = num_rows;
  n_cols = num_cols;
  curr_cell = start;
  target_cell = target;
  curr_dir = dir;
  
  // Initialise empty maze 
  // Set up horizontal walls
  h_walls = vector<vector<bool>>(n_rows+1, vector<bool>(n_cols, false));
  h_walls[0] = vector<bool>(n_cols, true);
  h_walls[n_rows] = vector<bool>(n_cols, true);
  
  // Set up vertical walls
  vector<bool> v_walls_row(n_cols+1, false);
  v_walls_row[0] = true;
  v_walls_row[n_cols] = true;
  v_walls = vector<vector<bool>>(n_rows, v_walls_row);
  
  // Set up cell values (distances from target)
  cell_vals = vector<vector<int>>(n_rows, vector<int>(n_cols, MAX_VAL));
  cell_vals[target_cell.row-1][target_cell.col-1] = 0;  
}

bool FloodFillMap::validCell (Cell c) {
  if (c.row < 1 || c.row > n_rows) return false;
  if (c.col < 1 || c.col > n_cols) return false;
  return true;
}
  
int FloodFillMap::getCellValue (Cell c) {
  if (!validCell(c)) return -1;
  return cell_vals[c.row - 1][c.col - 1];
}

Cell FloodFillMap::getNeighbourCell (Cell c, Direction dir) {
  Cell neighbour;
  switch (dir) {
    case north:
      neighbour = {c.row - 1, c.col}; break;
    case east:
      neighbour = {c.row, c.col + 1}; break;
    case south:
      neighbour = {c.row + 1, c.col}; break;
    case west:
      neighbour = {c.row, c.col - 1}; break;
  }
  return neighbour;
}

int FloodFillMap::setCellValue (Cell c, int value) {
  if (!validCell(c)) return -1;
  cell_vals[c.row - 1][c.col - 1] = value;
  return value;
}

bool FloodFillMap::wallIsPresent (Cell c, Direction dir) {
  if (!validCell(c)) {
    cout << "Invalid row or col for wallIsPresent!";
    return false;
  }
  switch (dir) {
    case north: return h_walls[c.row-1][c.col-1];
    case south: return h_walls[c.row][c.col-1];
    case east:  return v_walls[c.row-1][c.col];
    case west:  return v_walls[c.row-1][c.col-1];
    default:
      cout << "Invalid direction passed to wallIsPresent!";
      return false;
  }
}

vector<Cell> FloodFillMap::getNeighbourCells(Cell c) {
  vector<Cell> neighbours;
  for (auto direction_it = all_directions.begin();
            direction_it != all_directions.end();
            direction_it++) {
    if (!wallIsPresent(c,*direction_it)) {
      neighbours.push_back(getNeighbourCell(c,*direction_it));
    }
  }
  return neighbours;
}  

void FloodFillMap::doFloodFill() {
  int curr_explored_val = 0;
  bool maze_val_changed = true;
  //int iteration = 1;
  while (maze_val_changed) {
    maze_val_changed = false;
    //cout << "\tRunning iteration " << to_string(iteration++) << '\n';
    for (int r = 1; r <= n_rows; r++) {
      for (int c = 1; c <= n_cols; c++) {
        Cell curr = {r, c};
        int curr_cell_val = getCellValue(curr);
        if (curr_cell_val == curr_explored_val) {
          auto neighbours = getNeighbourCells(curr);
          for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++){
            Cell neighbour = *neighbour_it;
            if (getCellValue(neighbour) == MAX_VAL) {
              setCellValue(neighbour, curr_cell_val + 1);
              maze_val_changed = true;
            }
          }
        }
      }
    }
    curr_explored_val++;
  }
  return;
}

void FloodFillMap::updateFloodFill(Cell initial) {
  stack<Cell> c_stack;
  c_stack.push(initial);
  while (!c_stack.empty()) {
    Cell curr = c_stack.top();
    c_stack.pop();
    int min_neighbour_dist = MAX_VAL;
    auto neighbours = getNeighbourCells(curr);
    for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++){
      Cell neighbour = *neighbour_it;
      int neighbour_dist = getCellValue(neighbour);
      if (neighbour_dist < min_neighbour_dist) {
        min_neighbour_dist = neighbour_dist;
      }
    }
    if (min_neighbour_dist != getCellValue(curr) - 1) {
      setCellValue(curr, min_neighbour_dist + 1);
      for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++) {
        c_stack.push(*neighbour_it);
      }
    }      
  }
  return;
}

vector<Path> FloodFillMap::findShortestPaths(Cell c) {

  //fstream debug_fs;
  //debug_fs.open("../../debug.txt", ios::out);

  vector<Path> paths;
  
  // Check validity of starting cell
  if (!validCell(c)) {
    Path empty_path;
    paths.push_back(empty_path);
    //debug_fs << "Invalid cell passed to findShortestPaths\n";
    //debug_fs.close();
    return paths;
  }

  // Check if already at destination
  int curr_val = getCellValue(c);
  if (curr_val == 0) {
    Path empty_path;
    paths.push_back(empty_path);
    //debug_fs << "findShortestPaths reached destination\n";
    //debug_fs.close();
    return paths;
  }
  
  //debug_fs << "Checking cell at estimated distance: " << to_string(curr_val) << "\n";
  // Recursively build up paths by combining paths from neighbours
  for (auto dir_it = all_directions.begin(); dir_it != all_directions.end(); dir_it++) {
    if (wallIsPresent(c,*dir_it)) continue;
    Cell neighbour = getNeighbourCell(c,*dir_it);
    if (getCellValue(neighbour) == curr_val - 1) {
      auto sub_paths = findShortestPaths(neighbour);
      for (auto path = sub_paths.begin(); path != sub_paths.end(); path++) {
        path->push_back(*dir_it);
        paths.push_back(*path);
      }
    }
  }
  //debug_fs.close();
  return paths;
}

Path FloodFillMap::fewestTurnsPath(Cell c, Direction dir) {
  auto paths = findShortestPaths(c);
  //cout << "Shortest paths found!\n";
  Path fewest_turns_path = paths[0];
  for (auto path_it = paths.begin(); path_it != paths.end(); path_it++) {
    if (numTurnsInPath(*path_it, dir) <
        numTurnsInPath(fewest_turns_path, dir))
    {
      fewest_turns_path = *path_it;
    }
  }
  return fewest_turns_path;
}

void FloodFillMap::highlightPath(Path p) {
  // WARNING : This modifies the values of the maze
  int index = p.size();
  Cell curr = curr_cell;
  for (int r = 1; r <= n_rows; r++) {
    for (int c = 1; c <= n_cols; c++) {
      Cell to_empty = {r,c};
      setCellValue(to_empty, MAX_VAL); // Empty all cell values
    }
  }
  for (auto dir_it = p.rbegin(); dir_it != p.rend(); dir_it++) {
    Cell next = getNeighbourCell(curr, *dir_it);
    setCellValue(next, --index); // Refill values on the path
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
          if (curr.row == curr_cell.row && curr.col == curr_cell.col) {
            // Display starting position
            row_string.append(" " + dir2String(curr_dir) + " ");
          } else if (getCellValue(curr) == MAX_VAL) {
            // Display empty cell
            row_string.append("   ");
          } else {
            // Display path index of cell
            string value = to_string(getCellValue(curr));
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
    cout << row_string << '\n';
  }
}

void FloodFillMap::addWall(Cell c, Direction dir) {
  // Check validity
  if (!validCell(c)) {
    cout << "Invalid row or col for addWall!\n";
    return;
  }
  
  // Check if change is required
  if (wallIsPresent(c, dir)) {
    return;
  }
  
  // Update wall in map
  switch (dir) {
    case north:
      h_walls[c.row-1][c.col-1] = true; break;
    case south:
      h_walls[c.row][c.col-1] = true; break;
    case east: 
      v_walls[c.row-1][c.col] = true; break;
    case west:
      v_walls[c.row-1][c.col-1] = true; break;
    default:
      cout << "Invalid direction passed to addWall!\n";
  }
  
  // Update distance to target - modified floodfill
  //updateFloodFill(c);
  
  return;
}

