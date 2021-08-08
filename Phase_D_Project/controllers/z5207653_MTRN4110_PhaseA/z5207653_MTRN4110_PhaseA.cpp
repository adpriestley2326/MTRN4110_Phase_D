/*
 * File:          z5207653_MTRN4110_PhaseA.cpp
 * Date:          15/06/2021
 * Description:   Controller of E-Puck for Phase A - Driving and Perception
 * Author:        Terry Lim (z5207653)
 * Platform:      Windows
 * Notes:
 * Potentially overkilled with 11 distance sensors and a camera.
 * 3 distance sensors are used for wall detection, could be replaced by averaging
 * other distance sensors' values.
 * 8 distance sensors are used for EKF update step. Can be removed/ disabled at a 
 * cost of lowering max speed due to slip uncertainty.
 * Camera points towards ground, and is 1cm off the ground. Used also for EKF updates
 * and works in absence of walls by detecting seams between grids, but does nothing 
 * if spinning on the spot or moving on a different surface.
 * Unfortunately, at the higher max speed of 6.28, the camera only captures one frame
 * of the seam between cells.

 * Possible improvement:
 * Use proper EKF instead of fixed gain coefficients
 * Average values for more reliable wall detections. cycleCount and idleCount are in
 * place to be used for cyclical array storage and value averaging if needed later.
 * Could add wall avoidance controls, but they are not necessary with given specs.
 */
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

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
const std::string MESSAGE_PREFIX = "[z5207653_MTRN4110_PhaseA] ";

const double dsRotation[8] 
      = {-1.0/12*M_PI, 1.0/12*M_PI, 5.0/12*M_PI, 7.0/12*M_PI,
         11.0/12*M_PI, 13.0/12*M_PI, 17.0/12*M_PI, 19.0/12*M_PI};                      

double char_to_heading(char letter);
std::string heading_to_string(double heading);
void move_forward(Matrix<double, 3, 1> &pose);
double getAverage(double *a, int length);
double clamp(double v, double hi, double lo);
bool detectBlackLine(const unsigned char *image, int width, int height);

using namespace webots;

int main(int argc, char **argv) {

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