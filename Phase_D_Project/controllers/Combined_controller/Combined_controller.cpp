// File:          Combined_controller.cpp
// Date:          20/08/21
// Description:   An integrated phase D solution (without extra features)
// Author:        Terry Lim, Adam Priestly, Hrithika Nayak
// Modifications:

#include <webots/Robot.hpp>

#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
//#include <vector>

#include <FloodFillMap.hpp>
#include "HatTrickController.hpp"

// Motion stuff
#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Dense>
using namespace Eigen;
using namespace webots;

const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const std::string MESSAGE_PREFIX = "[MTRN4110_PhaseD] ";

// Path planning stuff
using namespace std;
const string MAP_FILE_NAME = "../../MapBuilt.txt";
const string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const string OUTPUT_FILE_NAME = "../../Output.txt";
fstream output_fs;
fstream path_fs;

void myPrint(string s);
vector<char> store_row_string(string s);
//string dir2String (Direction d);
string cell2String (Cell c);
int numTurnsInPath(Path p, Direction initial);
string generateMotionPlan(Path p, Cell initial_cell, Direction initial_dir);

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
  // ------------------ FROM PHASE B - PATH PLANNING STUFF ----------------------
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
  output_fs.close();

  
  // --------------------- FROM PHASE A - MOTION STUFF -----------------------
  std::ifstream inFile;
  inFile.open(MOTION_PLAN_FILE_NAME);
  std::cout << MESSAGE_PREFIX << "Reading in motion plan from " + MOTION_PLAN_FILE_NAME + "...\n";
  std::stringstream strStream;
  strStream << inFile.rdbuf();
  std::string motionPlan = strStream.str(); 
  std::cout << MESSAGE_PREFIX << "Motion Plan: " << motionPlan << "\n";
  std::cout << MESSAGE_PREFIX << "Motion plan read in!\n";
  
  std::ofstream outFile;
  outFile.open(MOTION_EXECUTION_FILE_NAME, std::ofstream::out);
  
  // create robot and controller
  HatTrickController *robot = new HatTrickController(motionPlan, outFile);

  std::cout << MESSAGE_PREFIX << "Executing motion plan...\n";
  outFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
    
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step() != -1) {
    robot->doUpdate();
    if (robot->isComplete()) break;
  };

  // Enter here exit cleanup code.
  inFile.close();
  outFile.close();
  delete robot;
  return 0;
}

// ---------------- PHASE B HELPER FUNCTIONS & CLASS DEFINITIONS --------------------
void myPrint(string s) {
  string out_s = MESSAGE_PREFIX + s + "\n";
  
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