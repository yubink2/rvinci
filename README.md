# dVRK Virtual Measurement Toolkit
This repository is based on RVinci from https://github.com/simonleonard/rvinci and 3D interaction cursors from https://github.com/aleeper/interaction_cursor_3d.
If you want to use our interactive mode, it is dependent on interactive marker tutorials package from https://github.com/ros-visualization/visualization_tutorials.

# How to run 
Launch the endoscope camera and camera calibration.
```
roslaunch rvinci rviz_stereo_pipeline.launch rig_name:=jhu_daVinci
```
If only want to use MTM:
```
rosrun dvrk_robot dvrk_console_json -j console-MTML-MTMR.json
```
If want to teleoperate PSM:
```
rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop.json
```
Launch rvinci. We have 3 different modes to do so.
To launch rvinci:
```
roslaunch rvinci rvinci_rviz.launch
```
To launch rvinci with buttons to select the mode for the measurement tool:
```
roslaunch rvinci rvinci_with_button.launch
```
To launch rvinci with interactive mode:
```
roslaunch rvinci rvinci_interactive.launch
```
