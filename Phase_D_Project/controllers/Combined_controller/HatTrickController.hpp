#pragma once
#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

// Motion stuff
#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Dense>
#include "FloodFillMap.hpp"

using namespace Eigen;
using namespace webots;

const double dsRotation[8] 
      = {-1.0/12*M_PI, 1.0/12*M_PI, 5.0/12*M_PI, 7.0/12*M_PI,
         11.0/12*M_PI, 13.0/12*M_PI, 17.0/12*M_PI, 19.0/12*M_PI};    

class HatTrickController {
  public:
  // Primary Functions
  HatTrickController(std::string motionPlan, std::ofstream &outFile);
  ~HatTrickController();
  int step();
  void doUpdate();
  int getIdleCount();
  void setIdleCount(int count);
  bool isComplete();
  protected:
  // Helper Functions
  void initialiseSensors();
  void predictionStep();
  void updateStepCamera();
  void updateStepDistanceSensors();
  void updateWheelVelocities();
  std::string heading_to_string(double heading);
  double char_to_heading(char letter);
  // move_forward is the function that sets the target pose for each 'F' move
  void move_forward(Matrix<double, 3, 1> &pose);
  double getAverage(double *a, int length);
  double clamp(double v, double lo, double hi);
  bool detectBlackLine(const unsigned char *image, int width, int height);
  void manualDrive();
  // Map-related
  Cell getCell();
  Direction heading_to_dir(double heading);
  bool visitedCell(Cell pos);

  // Constants
  const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
  const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
  const std::string MESSAGE_PREFIX = "[MTRN4110_PhaseD] ";
  const int TIME_STEP = 32;
  const double CELL_WIDTH = 0.165;
  const double WHEEL_RADIUS = 0.02;
  const double AXLE_LENGTH = 0.056*1.003;
  const double MAX_SPEED = 6.28;
  const int NUMBER_OF_DISTANCE_SENSORS = 8;
  const double DS_RANGE = 0.15;
  const int DS_MAX = 1500;
  const double WALL_THICK = 0.015;
  const int GROUND_CAM_HEIGHT = 128;
  const int GROUND_CAM_WIDTH = 128;
  const std::string actionList = "FLR";
  const unsigned char *groundCamImage;

  // Constants to tune
  const double MAX_OMEGA = MAX_SPEED/2.0;
  const int WALL_THRESHOLD = 1200;
  
  double kp = 5;
  double kpw = 3; // multiplier on bearing error
  double acceptablePositionError = 0.01;
  double acceptableRotationError = 0.5*M_PI/180.0;
  double acceptableRotationError2 = 0.5*M_PI/180.0; // used for enabling forward movement to next target
  double kcamera = 0.2;
  
  // EKF Stuff
  Matrix<double, 2, 2> ekfR;
  Matrix<double, 3, 3> ekfQ;
  Matrix<double, 3, 3> ekfP = MatrixXd::Zero(3,3);
  Matrix<double, 3, 3> ekfJ;
  Matrix<double, 3, 3> ekfFk;
  Matrix<double, 3, 3> ekfPu;
  Matrix<double, 2, 3> ekfH;
  Matrix<double, 2, 1> ekfZ;

  // Internal Variables
  std::ofstream &outFile;
  Robot *robot;
  Camera *groundCam;
  std::string motionPlan;
  Matrix<double, 3, 1> pose;
  Matrix<double, 3, 1> target;
  Matrix<double, 3, 1> targetCentre;
  Matrix<double, 3, 2> invKMatrix;
  double setLeftVelocity = 0;
  double setRightVelocity = 0;
  int action = 0;
  unsigned int motionPlanStep = 2;
  int cycleCount = 0;
  int idleCount = 0;
  bool mapped = false;
  bool speedrun = false;
  bool complete = false;
  bool positioned = false; 
  bool rotated = false;
  Motor *leftMotor;
  Motor *rightMotor;
  DistanceSensor *dsSensors[8];
  double dsValues[8];
  DistanceSensor *dsLeft;
  DistanceSensor *dsRight;
  DistanceSensor *dsFront;
};
