# MTRN4110_Phase_D - Hat Trick

## Base Features Run Instructions (Windows)
1. Add webots to path environment variables
2. Open Combined_World.wbt in worlds folder, use Webots to make clean and compile code again then close Webots
3. Set up Anaconda as described in this https://github.com/drliaowu/MTRN4110_21T2_Python_Tutorials/blob/master/Tutorial-1-Installing-python.md
4. Run ```activate mtrn4110``` and ```jupyter notebook```
5. Open Phase_D_base.ipynb in jupyter notebook
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


As of 08/08/2021
- Combined_controller is the basic combination of Phase A & B - barely modified so far
- The controller `Phase_D_dyanmic_planning` is for Adam's intial attempt at dynamically updating the path upon detecting an unforseen wall
- `Eigen` folder has been copied into the combined_controller's directory for the Phase A stuff to work
- Phase C vision `.ipynb` notebook is in the `controllers` folder
- 3 worlds currently in the `worlds` folder: two from phase C for getting the target, once from Terry's phase A for the adjustments to the robot - ideally we want to combine these into world files that work for the whole of Phase D
