#include "HatTrickController.hpp"
#include "helper.hpp"

HatTrickController::HatTrickController(std::string motionPlan) {
  this->motionPlan = motionPlan;
  // Do floodfill on a map without walls
  start = {1,1}; // My cell convention starts at 1, not 0
  endGoal = {3,5};
  currentGoal = {3,5};
  start_dir = south;
  this->map = new FloodFillMap(5, 9, start, currentGoal, start_dir);
  map->doFloodFill();
  // create the Robot instance.
  this->robot = new Robot();
  this->keyboard = new Keyboard();
  keyboard->enable(TIME_STEP);
  this->initialiseSensors();

  // initialise pose and target
  pose << (motionPlan[1] - '0')*CELL_WIDTH, 
          -(motionPlan[0] - '0')*CELL_WIDTH, 
          char_to_heading(motionPlan[2]);
  target = pose;
  targetCentre = pose;

  invKMatrix << WHEEL_RADIUS/2,            WHEEL_RADIUS/2,
                0,                         0,
                -WHEEL_RADIUS/AXLE_LENGTH, WHEEL_RADIUS/AXLE_LENGTH;
                
  // EKF
  // Uncertainty in observations
  ekfR << pow(0.005,2), 0, 
          0, pow(0.01,2);
  // Uncertainty in model prediction
  ekfQ = MatrixXd::Zero(3,3);
  ekfQ(0,0) = pow(0.01*TIME_STEP/1000.0, 2);
  ekfQ(1,1) = ekfQ(0,0);
  ekfQ(2,2) = MAX_OMEGA*ekfQ(0,0);
}

HatTrickController::~HatTrickController() {
  delete robot;
}

void HatTrickController::initialiseSensors() {
  // set up motors 
  leftMotor = robot->getMotor("left wheel motor");
  rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  // enable all sensors
  this->groundCam = robot->getCamera("groundCam");
  groundCam->enable(TIME_STEP);
  DistanceSensor *groundSensor1 = robot->getDistanceSensor("ground 1");
  groundSensor1->enable(TIME_STEP);
  dsLeft = robot->getDistanceSensor("DS_LEFT");
  dsRight = robot->getDistanceSensor("DS_RIGHT");
  dsFront = robot->getDistanceSensor("DS_FRONT");
  dsLeft->enable(TIME_STEP);
  dsRight->enable(TIME_STEP);
  dsFront->enable(TIME_STEP);
  
  for (int i = 0; i < 8; i++) {
    dsSensors[i] = robot->getDistanceSensor("ds"+std::to_string(i));
    dsSensors[i]->enable(TIME_STEP);
  }
}

int HatTrickController::step() {
  return (this->robot)->step(TIME_STEP);
}

void HatTrickController::doUpdate() {
  this->predictionStep();
  this->updateStepCamera();
  this->updateStepDistanceSensors();
  if (!speedrun) {
    int c = -1;
    do {
        c = this->keyboard->getKey();
        if (c == -1)
            break;
        autopilot = false;
        switch (c) {
            case ('A'):
            idleCount++;
            autopilot = true;
            break;
            case (keyboard->UP):
            speed = MAX_SPEED/10;
            yaw_speed = 0;
            break;
            case (keyboard->DOWN):
            speed = -MAX_SPEED/10;
            yaw_speed = 0;
            break;
            case (keyboard->RIGHT):
            yaw_speed = -MAX_OMEGA;
            speed = 0;
            break;
            case (keyboard->LEFT):
            yaw_speed = MAX_OMEGA;
            speed = 0;
            break;
            case ' ':
            speed = 0;
            yaw_speed = 0;
            break;
        }
    } while (c);
  }
  if (autopilot) this->updateWheelVelocities();
  else this->manualDrive(); 
  if (idleCount >= 1 && !mapped) {
    motionPlanStep++;
    // Print current state
    map->display();
    std::cout << MESSAGE_PREFIX << "Step: " << std::setw(3) << std::setfill('0') << motionPlanStep - 3
              << ", Row: " << (int)round(-pose(1)/CELL_WIDTH) << ", Column: " << (int)round(pose(0)/CELL_WIDTH)
              << ", Heading: " << heading_to_string(pose(2))
              << ", Left Wall: " << ((dsLeft->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << ", Front Wall: " <<  ((dsFront->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << ", Right Wall: " << ((dsRight->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << "\n";
    Cell pos = this->getCell();
    Direction dir = heading_to_dir(pose(2));

    std::vector<Cell> adjacents = map->getNeighbourCells(pos);

    if (dsLeft->getValue() < WALL_THRESHOLD) map->addWall(pos, heading_to_dir(pose(2)+M_PI/2));
    if (dsFront->getValue() < WALL_THRESHOLD) map->addWall(pos, heading_to_dir(pose(2)));
    if (dsRight->getValue() < WALL_THRESHOLD) map->addWall(pos, heading_to_dir(pose(2)-M_PI/2));
    
    map->updateFloodFill(pos);
    for (unsigned int i = 0; i < adjacents.size(); i++) map->updateFloodFill(adjacents[i]);

    std::vector<Cell> visitable = map->getNeighbourCells(pos);
    for (unsigned int i = 0; i < visitable.size(); i++) {
        if (!visitedCell(visitable[i])) {
            visited.push_back(visitable[i]);
            toVisit.push_back(visitable[i]);
            //std::cout << "adding " << visitable[i].col << visitable[i].row << "\n"; 
        }
    }

    for (auto it = toVisit.begin(); it != toVisit.end();) {
        Cell temp = *it;
        if (temp.row == pos.row && temp.col == pos.col) {
            toVisit.erase(it);
            //std::cout << "adding " << visitable[i].col << visitable[i].row << "\n"; 
        } else it++;
    }
    //std::cout << "now at " << pos.col << pos.row << "\n"; 

    if (pos.row == currentGoal.row && pos.col == currentGoal.col) {
        if (toVisit.size() != 0) {
            currentGoal = toVisit.back();
            toVisit.pop_back();
            map->changeTarget(pos, dir, currentGoal);
        } else {
            if (pos.row != start.row || pos.col != start.col) {
               currentGoal = start; 
               map->changeTarget(pos, dir, currentGoal);
            } else if (target(2) != char_to_heading(motionPlan[2])) {
                target(2) = char_to_heading(motionPlan[2]);
                pose(2) = fmod(pose(2), 2*M_PI);
                idleCount = 0;
                return;
            } else {
                mapped = true;
                std::cout << MESSAGE_PREFIX << "Finished Exploring!\n";
            }
        }
    } else {
        map->setPosition(pos, dir);
    }

    if ((target-pose).norm() > CELL_WIDTH*1.1) {
        Cell temp = toVisit.back();
        toVisit.pop_back();
        toVisit.push_back(currentGoal);
        currentGoal = temp;
        target = pose;
        map->changeTarget(pos, dir, currentGoal);
    } 

    // Advance to next step
    if (!mapped) {
        action = map->getAction(dir);
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
    }
    positioned = false;
    rotated = false;
    idleCount = 0;
  } else if (idleCount > 0 && !speedrun) {
        speedrun = true;
        motionPlanStep = 2;
        map->changeTarget(start, start_dir, endGoal);
        this->regenerateMotionPlan();
        std::cout << "Motion Plan: " << motionPlan << "\n";
        pose << (motionPlan[1] - '0')*CELL_WIDTH, 
          -(motionPlan[0] - '0')*CELL_WIDTH, 
          char_to_heading(motionPlan[2]);
        target = pose;
        kp = 15;
        kpw = 5; // multiplier on bearing error
        acceptablePositionError = 0.01;
        acceptableRotationError = 15*M_PI/180.0;
        acceptableRotationError2 = 35*M_PI/180.0; // used for enabling forward movement to next target

  }
  if (idleCount > 0 && speedrun) {
      std::cout << MESSAGE_PREFIX << "Step: " << std::setw(3) << std::setfill('0') << motionPlanStep - 2
              << ", Row: " << (int)round(-pose(1)/CELL_WIDTH) << ", Column: " << (int)round(pose(0)/CELL_WIDTH)
              << ", Heading: " << heading_to_string(pose(2))
              << ", Left Wall: " << ((dsLeft->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << ", Front Wall: " <<  ((dsFront->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << ", Right Wall: " << ((dsRight->getValue() < WALL_THRESHOLD) ? "Y":"N")
              << "\n";
    do {
      motionPlanStep++;
      if (motionPlanStep < motionPlan.length()) {
      // Advance to next step if one exists, else complete
        action = actionList.find(motionPlan[motionPlanStep]);
        bool lastF = false;
        std::string key ("F");
        std::size_t found = motionPlan.rfind(key);
        if (found != std::string::npos && motionPlanStep == found) lastF = true;
        switch(action) {
          case(0):
            target = targetCentre;
            move_forward(targetCentre);
            if (!lastF) target = (target+targetCentre)/2;
            else target = targetCentre;
            target(2) = targetCentre(2);
            break;
          case(1):
            targetCentre(2) += M_PI/2;
            continue;
          case(2):
            targetCentre(2) -= M_PI/2;
            continue;
        }
      } else {
          complete = true;
          std::cout << MESSAGE_PREFIX << "Motion plan executed!\n";
          break;
      }
    } while (action != 0);
    positioned = false;
    rotated = false;      
    idleCount = 0;
    this->updateWheelVelocities();
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
  if (complete) {
    setLeftVelocity = 0;
    setRightVelocity = 0;
  }
  leftMotor->setVelocity(setLeftVelocity);
  rightMotor->setVelocity(setRightVelocity);
  cycleCount++;
}

void HatTrickController::predictionStep() {
  double forwardVelocity = WHEEL_RADIUS*(setLeftVelocity + setRightVelocity)/2;
  double dt = TIME_STEP/1000.0;
  pose(0) += forwardVelocity*cos(pose(2))*dt;
  pose(1) += forwardVelocity*sin(pose(2))*dt;
  pose(2) += WHEEL_RADIUS*(-setLeftVelocity + setRightVelocity)/(AXLE_LENGTH) * dt;
  ekfJ << 1, 0, dt*forwardVelocity*sin(pose(2)),
          0, 1, dt*forwardVelocity*cos(pose(2)),
          0, 0, 1;
  // Covariance of process model
  ekfP = ekfJ*ekfP*ekfJ.transpose() + ekfQ;        
  // Covariance of process model inputs
  ekfFk << dt*cos(pose(2)), 0,
           dt*sin(pose(2)), 0,
           0, dt;
  ekfPu << pow(MAX_SPEED*WHEEL_RADIUS*2*0.05, 2), 0,
           0, pow(MAX_OMEGA*WHEEL_RADIUS*2*0.05/WHEEL_RADIUS, 2);
  ekfP = ekfP + ekfFk*ekfPu*ekfFk.transpose();
}

void HatTrickController::updateStepCamera() {
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
      pose(0) += (eCamX - seamOffset*cos(pose(2)) - pose(0))*0.05;
    } else {
      // Find rows and adjust y
      double eCamY = round((pose(1) + CELL_WIDTH/2 + seamOffset*sin(pose(2)))/CELL_WIDTH)*CELL_WIDTH;
      eCamY -= CELL_WIDTH/2;
      pose(1) += (eCamY - seamOffset*sin(pose(2)) - pose(1))*0.05;
    }
    pose(2) += atan2(rightRowPos-leftRowPos, rightColPos-leftColPos)*0.05; 
  }
}

void HatTrickController::updateStepDistanceSensors() {
  for(int i = 0; i < 8; i++) dsValues[i] = dsSensors[i]->getValue()*DS_RANGE/DS_MAX;
  // Update pose using distance sensors
  double errorRotation;
  Matrix<double, 2, 1> p1, p2;
  // skip if heading is not expected to be aligned
  // with cardinal directions to avoid corners being misidentified as edges.
  double headingTest = fmod(pose(2),M_PI/2);
  if (headingTest < 0) headingTest += M_PI/2;
  for (int d = 0; d < 4 && abs(M_PI/4 - headingTest) > 35*M_PI/180.0; d++) {
    // can't estimate well with only one ray
    if (dsValues[2*d] ==  DS_RANGE || dsValues[2*d+1] == DS_RANGE) continue;
    p1(0) = pose(0) + dsValues[2*d]*cos(pose(2) + dsRotation[2*d]);
    p1(1) = pose(1) + dsValues[2*d]*sin(pose(2) + dsRotation[2*d]);
    p2(0) = pose(0) + dsValues[2*d+1]*cos(pose(2) + dsRotation[2*d+1]);
    p2(1) = pose(1) + dsValues[2*d+1]*sin(pose(2) + dsRotation[2*d+1]);
    
    // Adjust rotation 
    errorRotation = atan2(p2(1)-p1(1), p2(0)-p1(0)) - M_PI/2 - dsRotation[2*d]/2 - dsRotation[2*d+1]/2 - pose(2);
    // Make error between -pi and pi
    errorRotation = fmod(errorRotation + M_PI, 2*M_PI);
    if (errorRotation < 0) errorRotation += 2*M_PI;
    errorRotation -= M_PI;
    
    
    // Adjust position
    int dir = ((int) round((pose(2) + dsRotation[2*d]/2 + dsRotation[2*d+1]/2)*2/M_PI))%4;
    if (dir < 0) dir += 4;
    Matrix<double, 2, 1> ekfY;
    if (dir % 2 == 0) { // Columns, x
      double eX = round((p1(0)/2 + p2(0)/2 - CELL_WIDTH/2)/CELL_WIDTH)*CELL_WIDTH + CELL_WIDTH/2;
      if (pose(0) < eX)  eX -= WALL_THICK/2;
      else eX += WALL_THICK/2;
      double errorX = (eX - (p1(0)/2 + p2(0)/2)) ;
      ekfH << 1, 0, -dsValues[2*d]*sin(pose(2) + dsRotation[2*d])/2 - dsValues[2*d+1]*sin(pose(2) + dsRotation[2*d+1])/2,
              0, 0, -1;
      ekfZ << errorX, errorRotation;     
    } else { // Rows, y
      double eY = round((p1(1)/2 + p2(1)/2 + CELL_WIDTH/2)/CELL_WIDTH)*CELL_WIDTH - CELL_WIDTH/2;
      if (pose(1) < eY)  eY -= WALL_THICK/2;
      else eY += WALL_THICK/2;
      double errorY = (eY - (p1(1)/2 + p2(1)/2));
      ekfH << 0, 1, dsValues[2*d]*cos(pose(2) + dsRotation[2*d])/2 + pose(1) + dsValues[2*d+1]*cos(pose(2) + dsRotation[2*d+1])/2,
              0, 0, -1;
      ekfZ << errorY, errorRotation;
    }
    // EKF Pose Updater
    Matrix<double, 2, 2> ekfS;
    ekfS = ekfR + ekfH*ekfP*ekfH.transpose();
    Matrix<double, 3, 2> ekfK;
    ekfK = ekfP*ekfH.transpose()*ekfS.inverse();
    pose = pose + ekfK*ekfZ;
    ekfP = ekfP-ekfK*ekfH*ekfP;
  }
}

void HatTrickController::updateWheelVelocities() {
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
    if ((target(2)-pose(2))*offsetAngle < 0 && abs(offsetAngle) > M_PI/3) offsetAngle*=-1;
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
}

int HatTrickController::getIdleCount() {
  return this->idleCount;
}

void HatTrickController::setIdleCount(int count) {
  this->idleCount = count;
}

bool HatTrickController::isComplete() {
  return this->complete;
}

void HatTrickController::manualDrive() {
    Matrix<double, 3, 1> columnVector(speed, 0, yaw_speed);
    Matrix<double, 2, 1> invKSolution = invKMatrix.colPivHouseholderQr().solve(columnVector);
    // Scale desired speed to remain within limits
    if (invKSolution.maxCoeff() > MAX_SPEED) invKSolution *= MAX_SPEED/(kp*invKSolution.maxCoeff());
    if (invKSolution.minCoeff() < -MAX_SPEED) invKSolution *= -MAX_SPEED/(kp*invKSolution.minCoeff());
    setLeftVelocity = kp*invKSolution(0);
    setRightVelocity = kp*invKSolution(1);
}

// -------------------- PHASE A HELPER FUNCTIONS ---------------------
std::string HatTrickController::heading_to_string(double heading) {
  std::string orientations[4] = {"E", "N", "W", "S"};
  int i = ((int) round(heading*2/M_PI))%4;
  if (i < 0) i += 4;
  return orientations[i]; 
}

double HatTrickController::char_to_heading(char letter) {
  std::string order = "ENWS";
  std::size_t pos = order.find(letter);
  if (pos != std::string::npos) return M_PI/2 * pos;
  else return 0;
}

// Note that moving down is decreasing Y.
void HatTrickController::move_forward(Matrix<double, 3, 1> &pose) {
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

double HatTrickController::getAverage(double *a, int length) {
  double sum = 0;
  for (int i = 0; i < length; i++) {
    sum += a[i];
  }
  return sum/length;
}

double HatTrickController::clamp(double v, double lo, double hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

// Returns true if and only if enough black is detected on both halves of the image.
bool HatTrickController::detectBlackLine(const unsigned char *image, int width, int height) {
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

Direction HatTrickController::heading_to_dir(double heading) {
  Direction orientations[4] = {east, north, west, south};
  int i = ((int) round(heading*2/M_PI))%4;
  if (i < 0) i += 4;
  return orientations[i]; 
}

Cell HatTrickController::getCell() {
  Cell pos;
  pos.row = -round(this->pose(1)/CELL_WIDTH)+1;
  pos.col = round(this->pose(0)/CELL_WIDTH)+1;
  return pos;
}

bool HatTrickController::visitedCell(Cell pos) {
  for (unsigned int i = 0; i < visited.size(); i++) {
    if (visited[i].row == pos.row && visited[i].col == pos.col) return true;
  }
  return false;
}

void HatTrickController::regenerateMotionPlan() {
  auto paths = map->findShortestPaths(map->start_cell);
  for (auto path_it = paths.begin(); path_it != paths.end(); path_it++) {
    map->highlightPath(*path_it);
    map->display();
  } 
  // Task 3a - Check for fewest turns
  Path fewest_turns_path = paths[0];
  for (auto path_it = paths.begin(); path_it != paths.end(); path_it++) {
    if (numTurnsInPath(*path_it, map->start_dir) <
        numTurnsInPath(fewest_turns_path, map->start_dir))
    {
      fewest_turns_path = *path_it;
    }
  }
  map->highlightPath(fewest_turns_path);
  map->display();
  
  this->motionPlan = generateMotionPlan(fewest_turns_path, map->start_cell, map->start_dir);
}