# MTRN4110_Phase_D - Hat Trick

## Base Features Run Instructions (Windows)
1. Add webots to path environment variables
2. Open Combined_World.wbt in worlds folder, use Webots to make clean and compile code again then close Webots
3. Set up Anaconda as described in this https://github.com/drliaowu/MTRN4110_21T2_Python_Tutorials/blob/master/Tutorial-1-Installing-python.md
4. Run ```activate mtrn4110``` and ```jupyter notebook```
5. Open Phase_D.ipynb in jupyter notebook
6. Run all cells

## Extra Features Run Instructions (Windows)
### Robot Controller improvements
The Improved_World.wbt world file contains the following features
- Extended Kalman Filter Localisation
    - Pose prediction updates using 8 distance sensors and one ground-facing camera.
- Maze exploration, mapping and planning
- Keyboard manual control
    - Resume exploration/autopilot
- Improved speed run path planning

### Motion Tracking
To make use of the motion tracking, a video recording of the robot navigating the maze must be made. The video must be cropped so that the corners of the frame correspond to the inner corners of the maze. This will provide the most accurate coordinates for tracking motion.
1. Save the video as 'MotionTracking.mp4'
2. Open Object_tracking.ipynb in jupyter notebook
3. Run all
4. Find the resulting video as 'MotionTrackingResult.avi'
