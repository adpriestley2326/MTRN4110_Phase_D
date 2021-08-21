// FloodFillMap.cpp

#include <iostream>
#include <string>
#include <FloodFillMap.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>

string dir2String (Direction d) {
  switch (d) {
    case north: return "^";
    case east: return ">";
    case south: return "v";
    case west: return "<";
  }
  return "o";
}

const string OUTPUT_FILE_NAME = "../../Output.txt";

vector<Direction> all_directions = {north,east,south,west};

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

FloodFillMap::FloodFillMap (int num_rows, int num_cols, Cell start, Cell target, Direction dir) {
  n_rows = num_rows;
  n_cols = num_cols;
  start_cell = start;
  target_cell = target;
  start_dir = dir;
  
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
    cout << "Invalid row or col for wallIsPresent!";
    return false;
  }
  switch (dir) {
    case north: return h_walls[row-1][col-1];
    case south: return h_walls[row][col-1];
    case east:  return v_walls[row-1][col];
    case west:  return v_walls[row-1][col-1];
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
    if (!wallIsPresent(c.row,c.col,*direction_it)) {
      neighbours.push_back(getNeighbourCell(c.row,c.col,*direction_it));
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
        int curr_cell_val = getCellValue(r,c);
        if (curr_cell_val == curr_explored_val) {
          Cell curr = {r, c};
          auto neighbours = getNeighbourCells(curr);
          for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++){
            Cell neighbour = *neighbour_it;
            if (getCellValue(neighbour.row, neighbour.col) == MAX_VAL) {
              setCellValue(neighbour.row, neighbour.col, curr_cell_val + 1);
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
  if (initial.row == target_cell.row && initial.col == target_cell.col) return;
  stack<Cell> c_stack;
  c_stack.push(initial);
  while (!c_stack.empty()) {
    Cell curr = c_stack.top();
    c_stack.pop();
    int min_neighbour_dist = MAX_VAL;
    auto neighbours = getNeighbourCells(curr);
    for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++){
      Cell neighbour = *neighbour_it;
      int neighbour_dist = getCellValue(neighbour.row, neighbour.col);
      if (neighbour_dist < min_neighbour_dist) {
        min_neighbour_dist = neighbour_dist;
      }
    }
    if (min_neighbour_dist != getCellValue(curr.row, curr.col) - 1) {
      setCellValue(curr.row, curr.col, min_neighbour_dist + 1);
      for (auto neighbour_it = neighbours.begin(); neighbour_it != neighbours.end(); neighbour_it++) {
        c_stack.push(*neighbour_it);
      }
    }      
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
      setCellValue(r, c, MAX_VAL); // Empty all cell values
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
          } else if (getCellValue(curr.row, curr.col) == MAX_VAL) {
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
    cout << row_string << '\n';
  }
}

bool FloodFillMap::addWall(Cell c, Direction dir) {
  // Check validity
  if (!validRowCol(c.row,c.col)) {
    cout << "Invalid row or col for addWall!\n";
    return false;
  }
  
  // Check if change is required
  if (wallIsPresent(c.row, c.col, dir)) {
    return false;
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
  
  return true;
}

void FloodFillMap::resetValues() {
  for (int r = 1; r < 6; r++) {
    for (int c = 1; c < 10; c++) this->setCellValue(r, c, MAX_VAL);
  }
  setCellValue(target_cell.row, target_cell.col, 0);
}

void FloodFillMap::setPosition(Cell new_start, Direction dir) {
  this->start_cell = new_start;
  this->start_dir = dir;
}

void FloodFillMap::changeTarget(Cell new_start, Direction dir, Cell new_target) {
  this->start_cell = new_start;
  this->start_dir = dir;
  this->target_cell = new_target;
  this->resetValues();
  this->doFloodFill();
}

int FloodFillMap::getAction(Direction d) {
  int currentVal = getCellValue(start_cell.row, start_cell.col);
  if (!wallIsPresent(start_cell.row, start_cell.col, d)) {
    Cell neighbour = getNeighbourCell(start_cell.row, start_cell.col, d);
    if (getCellValue(neighbour.row, neighbour.col) == currentVal-1) return 0;
  }
  if (!wallIsPresent(start_cell.row, start_cell.col, Direction((d+1)%4))) {
    Cell neighbour = getNeighbourCell(start_cell.row, start_cell.col, Direction((d+1)%4));
    if (getCellValue(neighbour.row, neighbour.col) == currentVal-1) return 2; 
  }
  return 1;
}