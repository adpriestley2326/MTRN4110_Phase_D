 // File:          Improved_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
//#include <vector>

#include "HatTrickController.hpp"
#include "helper.hpp"
#include <FloodFillMap.hpp>

using namespace webots;

const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const std::string MESSAGE_PREFIX = "[MTRN4110_PhaseD] ";

// Path planning stuff
using namespace std;
void myPrint(string s);

const string MAP_FILE_NAME = "../../MapBuilt.txt";
const string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const string OUTPUT_FILE_NAME = "../../Output.txt";
fstream output_fs;
fstream path_fs;

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // ------------------ PATH PLANNING STUFF ----------------------
  // Open files for writing
  output_fs.open(OUTPUT_FILE_NAME, ios::out);
  path_fs.open(PATH_PLAN_FILE_NAME, ios::out);

  // Open map file
  myPrint("Reading in map from " + MAP_FILE_NAME + "...");
  fstream map_fs(MAP_FILE_NAME, ios::in);
  if (!map_fs.is_open()) {
    myPrint("Could not read map file!");
    return 1;
  }
  
  // Read map file
  string map_line;
  vector<vector<char>> char_map;
  while (getline(map_fs, map_line)) {
    //myPrint(map_line);
    vector<char> row = store_row_string(map_line);
    char_map.push_back(row);
  }
  map_fs.close();
  myPrint("Map read in!");
  
  
  // Do floodfill on a map without walls
  Cell start = {1,1}; // My cell convention starts at 1, not 0
  Cell target = {3,5};
  Direction dir = south;
  FloodFillMap map(5, 9, start, target, dir);
  myPrint("Map created successfully!");
  myPrint("Running floodfill...");
  map.doFloodFill();
  myPrint("Floodfill successful!");
  map.display();
  
  // Add wall and test path updating
  myPrint("Simulating detected wall at cell (3,4) in east direction...");
  Cell target_left = {target.row, target.col-1};
  map.addWall(target_left, east);
  map.updateFloodFill(target_left);
  myPrint("Updated map:");
  map.display();
  
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
  HatTrickController *robot = new HatTrickController(motionPlan);

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