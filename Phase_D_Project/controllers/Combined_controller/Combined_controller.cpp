// File:          Combined_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>


// Motion stuff
#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Dense>
using namespace Eigen;

#define TIME_STEP 64
#define CELL_WIDTH 0.165
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.0566*1.003
#define MAX_SPEED 6.28
#define MAX_OMEGA 6.28/3
#define NUMBER_OF_DISTANCE_SENSORS 8
#define DS_RANGE 0.15
#define DS_MAX 1500
#define WALL_THRESHOLD 1200
#define WALL_THICK 0.015
#define GROUND_CAM_WIDTH 128
#define GROUND_CAM_HEIGHT 128

const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const std::string MESSAGE_PREFIX = "[MTRN4110_PhaseD] ";


// Path planning stuff
using namespace std;
#define N 200

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

const double dsRotation[8] 
      = {-1.0/12*M_PI, 1.0/12*M_PI, 5.0/12*M_PI, 7.0/12*M_PI,
         11.0/12*M_PI, 13.0/12*M_PI, 17.0/12*M_PI, 19.0/12*M_PI};                      

double char_to_heading(char letter);
std::string heading_to_string(double heading);
void move_forward(Matrix<double, 3, 1> &pose);
double getAverage(double *a, int length);
double clamp(double v, double hi, double lo);
bool detectBlackLine(const unsigned char *image, int width, int height);

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
  
  // create the Robot instance.
  Robot *robot = new Robot();
 
  // set up motors 
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  double setLeftVelocity = 0;
  double setRightVelocity = 0;
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  
  // enable all sensors
  Camera *groundCam = robot->getCamera("groundCam");
  groundCam->enable(TIME_STEP);
  DistanceSensor *groundSensor1 = robot->getDistanceSensor("ground 1");
  groundSensor1->enable(TIME_STEP);
  DistanceSensor *dsLeft = robot->getDistanceSensor("DS_LEFT");
  DistanceSensor *dsRight = robot->getDistanceSensor("DS_RIGHT");
  DistanceSensor *dsFront = robot->getDistanceSensor("DS_FRONT");
  dsLeft->enable(TIME_STEP);
  dsRight->enable(TIME_STEP);
  dsFront->enable(TIME_STEP);
  
  DistanceSensor *dsSensors[8];
  for (int i = 0; i < 8; i++) {
    dsSensors[i] = robot->getDistanceSensor("ds"+std::to_string(i));
    dsSensors[i]->enable(TIME_STEP);
  }
  double dsValues[8];
 
  // List of actions and durations of action
  // Front, Left, Right
  int action = 0;
  std::string actionList = "FLR";
   
  // initialise pose and target
  Matrix<double, 3, 1> pose;
  pose << (motionPlan[1] - '0')*CELL_WIDTH, 
          -(motionPlan[0] - '0')*CELL_WIDTH, 
          char_to_heading(motionPlan[2]);
  
  Matrix<double, 3, 1> target = pose;
  
  unsigned int motionPlanStep = 2;

  int cycleCount = 0;
  int idleCount = 0;
  bool complete = false;
  double kp = 5;
  double kpw = 3; // multiplier on bearing error
  double acceptablePositionError = 0.01;
  double acceptableRotationError = 0.5*M_PI/180.0;
  double acceptableRotationError2 = 3*M_PI/180.0; // used for enabling forward movement to next target
  Matrix<double, 3, 2> invKMatrix;
  invKMatrix << WHEEL_RADIUS/2,            WHEEL_RADIUS/2,
                0,                         0,
                -WHEEL_RADIUS/AXLE_LENGTH, WHEEL_RADIUS/AXLE_LENGTH;
                
  const unsigned char *groundCamImage;
  std::cout << MESSAGE_PREFIX << "Executing motion plan...\n";
  outFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
  
  // A boolean to ensure position is always corrected before heading
  // and prevents further position correction in event that changing heading
  // nudges position very very slightly outside acceptable range (prevents 
  // oscillation between position and heading correction).
  bool positioned = false; 
  // Boolean to enable forward movement only if the robot is facing
  // the right direction to reach its target.
  // Potentially usable in later phases to control motion using only 
  // points without specified heading.
  bool rotated = false;
  
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1 && !complete) {
    // Update internal model by euler method
    double forwardVelocity = WHEEL_RADIUS*(setLeftVelocity + setRightVelocity)/2;
    double dt = TIME_STEP/1000.0;
    pose(0) += forwardVelocity*cos(pose(2))*dt;
    pose(1) += forwardVelocity*sin(pose(2))*dt;
    pose(2) += WHEEL_RADIUS*(-setLeftVelocity + setRightVelocity)/(AXLE_LENGTH) * dt;
    
    // Read the camera
    groundCamImage = groundCam->getImage();
    // Update pose based on camera information if a seam is found
    if (detectBlackLine(groundCamImage, GROUND_CAM_WIDTH, GROUND_CAM_HEIGHT)) {
      // Use the average position of the black line on left and right halves of image
      // to generate a fitting line and estimate camera pose
      int leftNumBlack = 0;
      double leftRowPos = 0;
      double leftColPos = 0;
      int rightNumBlack = 0;
      double rightRowPos = 0;
      double rightColPos = 0;
      int width = GROUND_CAM_WIDTH;
      int height = GROUND_CAM_HEIGHT;
      int r,g,b;
      for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
          r = Camera::imageGetRed(groundCamImage, width, col, row);
          g = Camera::imageGetGreen(groundCamImage, width, col, row);
          b = Camera::imageGetBlue(groundCamImage, width, col, row);
          if (r + g + b < 80) {
            if (col < width/2) {
              leftNumBlack++;
              leftRowPos += row;  
              leftColPos += col;  
            } else {
              rightNumBlack++;
              rightRowPos += row;
              rightColPos += col;
            }  
          }
        }
      }
      // Failsafe, but the checks made earlier should ensure the divisor is not 0
      if (leftNumBlack <= 0) leftNumBlack = 1;
      if (rightNumBlack <= 0) rightNumBlack = 1;
      leftRowPos /= leftNumBlack;
      leftColPos /= leftNumBlack;
      rightRowPos /= rightNumBlack;
      rightColPos /= rightNumBlack;
      // Estimate camera pose
      // one of x or y will be along the gridlines (offset a little by position of the line)
      // Use row pos to determine where the gridline is wrt the robot
      // Use atan2 to determine the angle of the gridline wrt the robot
      // noting ground cam is 3cm infront of the robot's centre, view is about 8.5mm tall
      // i.e. top of cap is 3.425 cm infront, subtracting 8.5/127 for each row down.
      // take camera position, subtract cell/2 to align with grid, divide by cell, then round
      // to find expected grid line that matches.
      double seamOffset = 0.03425 - 0.0085/127*(leftRowPos+rightRowPos)/2;
      int i = ((int) round(pose(2)*2/M_PI))%4;
      if (i < 0) i += 4;
      if (i % 2 == 0) {
        // Find columns and adjust x
        double eCamX = round((pose(0) - CELL_WIDTH/2 + seamOffset*cos(pose(2)))/CELL_WIDTH)*CELL_WIDTH;
        eCamX += CELL_WIDTH/2;
        pose(0) += (eCamX - seamOffset*cos(pose(2)) - pose(0))*0.2;
      } else {
        // Find rows and adjust y
        double eCamY = round((pose(1) + CELL_WIDTH/2 + seamOffset*sin(pose(2)))/CELL_WIDTH)*CELL_WIDTH;
        eCamY -= CELL_WIDTH/2;
        pose(1) += (eCamY - seamOffset*sin(pose(2)) - pose(1))*0.2;
      }
      pose(2) += atan2(rightRowPos-leftRowPos, rightColPos-leftColPos)*0.2; 
    }
    
    for(int i = 0; i < 8; i++) dsValues[i] = dsSensors[i]->getValue()*DS_RANGE/DS_MAX;
    
    // Update pose using distance sensors
    double dsKRotation = 0.1;
    double dsKPosition = 0.05;
    double errorRotation;
    Matrix<double, 2, 1> p1, p2;
    // skip if heading is not expected to be aligned
    // with cardinal directions to avoid corners being misidentified as edges.
    double headingTest = fmod(pose(2),M_PI/2);
    if (headingTest < 0) headingTest += M_PI/2;
    for (int d = 0; d < 4 && abs(M_PI/4 - headingTest) > 40*M_PI/180.0; d++) {
      // can't estimate well with only one ray
      if (dsValues[2*d] ==  DS_RANGE || dsValues[2*d+1] == DS_RANGE) continue;
      p1(0) = pose(0) + dsValues[2*d]*cos(pose(2) + dsRotation[2*d]);
      p1(1) = pose(1) + dsValues[2*d]*sin(pose(2) + dsRotation[2*d]);
      p2(0) = pose(0) + dsValues[2*d+1]*cos(pose(2) + dsRotation[2*d+1]);
      p2(1) = pose(1) + dsValues[2*d+1]*sin(pose(2) + dsRotation[2*d+1]);
      
      // Adjust rotation 
      errorRotation = 0 - (atan2(p2(1)-p1(1), p2(0)-p1(0)) - M_PI/2 - dsRotation[2*d]/2 - dsRotation[2*d+1]/2 - pose(2));
      // Make error between -pi and pi
      errorRotation = fmod(errorRotation + M_PI, 2*M_PI);
      if (errorRotation < 0) errorRotation += 2*M_PI;
      errorRotation -= M_PI;
      pose(2) += dsKRotation*errorRotation;
      
      // Adjust position
      int dir = ((int) round((pose(2) + dsRotation[2*d]/2 + dsRotation[2*d+1]/2)*2/M_PI))%4;
      if (dir < 0) dir += 4;
      if (dir % 2 == 0) { // Columns, x
        double eX = round((p1(0)/2 + p2(0)/2 - CELL_WIDTH/2)/CELL_WIDTH)*CELL_WIDTH + CELL_WIDTH/2;
        if (pose(0) < eX)  eX -= WALL_THICK/2;
        else eX += WALL_THICK/2;
        pose(0) += dsKPosition*(eX - (p1(0)/2 + p2(0)/2));
      } else { // Rows, y
        double eY = round((p1(1)/2 + p2(1)/2 + CELL_WIDTH/2)/CELL_WIDTH)*CELL_WIDTH - CELL_WIDTH/2;
        if (pose(1) < eY)  eY -= WALL_THICK/2;
        else eY += WALL_THICK/2;
        pose(1) += dsKPosition*(eY - (p1(1)/2 + p2(1)/2));
      }
      
    }
       
    // Determine output based on closed loop feedback
    double positionError = (target-pose).segment(0,2).norm();
    if (positionError < acceptablePositionError) positioned = true;
    double rotationError = abs((target-pose)(2));
    if (positioned && rotationError < acceptableRotationError) {
      setLeftVelocity = 0;
      setRightVelocity = 0;
      idleCount++;
    } else {
      Matrix<double, 2, 1> forwardVector(cos(pose(2)), sin(pose(2)));
      forwardVector = forwardVector/forwardVector.norm();
      double errorDistance = (target-pose).segment(0,2).dot(forwardVector);
      // Angle required to allow robot to move towards the target x,y 
      double offsetAngle = atan2(target(1)-pose(1), target(0)-pose(0)) - pose(2);
      // obtain between -pi and pi then between -pi/2 and pi/2
      offsetAngle = std::fmod(offsetAngle + M_PI, 2*M_PI);
      if (offsetAngle < 0) offsetAngle += 2*M_PI;
      offsetAngle -= M_PI;
      if (offsetAngle < -M_PI/2) offsetAngle += M_PI;
      if (offsetAngle > M_PI/2) offsetAngle -= M_PI;
      // Angle required to rotate robot to correct rotation
      double errorAngle = 0;
      if (positioned) errorAngle = target(2) - pose(2);
      else errorAngle = offsetAngle;
      if (abs(errorAngle) < acceptableRotationError2) rotated = true;
      Matrix<double, 3, 1> columnVector(rotated ? errorDistance:0, 0, errorAngle*kpw);
      Matrix<double, 2, 1> invKSolution = invKMatrix.colPivHouseholderQr().solve(columnVector);
      // Scale desired speed to remain within limits
      if (invKSolution.maxCoeff() > MAX_SPEED) invKSolution *= MAX_SPEED/(kp*invKSolution.maxCoeff());
      if (invKSolution.minCoeff() < -MAX_SPEED) invKSolution *= -MAX_SPEED/(kp*invKSolution.minCoeff());
      setLeftVelocity = kp*invKSolution(0);
      setRightVelocity = kp*invKSolution(1);
    }
    
    // idleCount should equal number of values averaged, but
    // that is currently 1 to optimise speed since the sensors are
    // fairly consistent/accurate in simulation.
    if (idleCount >= 1) {
      motionPlanStep++;
      // Print current state
      std::cout << MESSAGE_PREFIX << "Step: " << std::setw(3) << std::setfill('0') << motionPlanStep - 3
                << ", Row: " << (int)round(-pose(1)/CELL_WIDTH) << ", Column: " << (int)round(pose(0)/CELL_WIDTH)
                << ", Heading: " << heading_to_string(pose(2))
                << ", Left Wall: " << ((dsLeft->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << ", Front Wall: " <<  ((dsFront->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << ", Right Wall: " << ((dsRight->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << "\n";
      outFile << motionPlanStep - 3
                << "," << (int)round(-pose(1)/CELL_WIDTH) << "," << (int)round(pose(0)/CELL_WIDTH)
                << "," << heading_to_string(pose(2))
                << "," << ((dsLeft->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << "," <<  ((dsFront->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << "," << ((dsRight->getValue() < WALL_THRESHOLD) ? "Y":"N")
                << "\n";

      if (motionPlanStep < motionPlan.length()) {
        // Advance to next step if one exists, else complete
        action = actionList.find(motionPlan[motionPlanStep]);
        switch(action) {
          case(0):
            move_forward(target);
            break;
          case(1):
            target(2) += M_PI/2;
            break;
          case(2):
            target(2) -= M_PI/2;
            break;
        } 
        positioned = false;
        rotated = false;
      } else {
        complete = true;
        std::cout << MESSAGE_PREFIX << "Motion plan executed!\n";
      }
      idleCount = 0;
    }
    
    
    // Do not allow an overall w > 2pi/3 as it slips too much
    double magW = abs(-setLeftVelocity + setRightVelocity)/2;
    if (magW > MAX_OMEGA) {
      setLeftVelocity *= MAX_OMEGA/magW;
      setRightVelocity *= MAX_OMEGA/magW;
    }
    // Clamp speeds to be safe
    setLeftVelocity = clamp(setLeftVelocity, -MAX_SPEED, MAX_SPEED);
    setRightVelocity = clamp(setRightVelocity, -MAX_SPEED, MAX_SPEED);
    leftMotor->setVelocity(setLeftVelocity);
    rightMotor->setVelocity(setRightVelocity);
    cycleCount++;
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


// --- FloodFillMap member functions -----

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

// -------------------- PHASE A HELPER FUNCTIONS ---------------------
std::string heading_to_string(double heading) {
  std::string orientations[4] = {"E", "N", "W", "S"};
  int i = ((int) round(heading*2/M_PI))%4;
  if (i < 0) i += 4;
  return orientations[i]; 
}

double char_to_heading(char letter) {
  std::string order = "ENWS";
  std::size_t pos = order.find(letter);
  if (pos != std::string::npos) return M_PI/2 * pos;
  else return 0;
}

// Note that moving down is decreasing Y.
void move_forward(Matrix<double, 3, 1> &pose) {
  int i = ((int) round(pose(2)*2/M_PI))%4;
  if (i < 0) i += 4;
  switch(i) {
    case(0):
      pose(0) += CELL_WIDTH; break;
    case(1):
      pose(1) += CELL_WIDTH; break;
    case(2):
      pose(0) -= CELL_WIDTH; break;
    case(3):
      pose(1) -= CELL_WIDTH; break;
  }
}

double getAverage(double *a, int length) {
  double sum = 0;
  for (int i = 0; i < length; i++) {
    sum += a[i];
  }
  return sum/length;
}

double clamp(double v, double lo, double hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

// Returns true if and only if enough black is detected on both halves of the image.
bool detectBlackLine(const unsigned char *image, int width, int height) {
  bool left = false;
  bool right = false;
  int numBlack = 0;
  int r,g,b;
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      r = Camera::imageGetRed(image, width, col, row);
      g = Camera::imageGetGreen(image, width, col, row);
      b = Camera::imageGetBlue(image, width, col, row);
      if (r + g + b < 80) {
        numBlack++;
        if (col < width/2) left = true;
        else right = true;
      }
    }
  }
  // Something's wrong if more than an eighth of the camera is black
  if (numBlack > width*height/8) return false;
  return ((numBlack > width) && left && right) ? true:false;
}