# dVRK Virtual Measurement Toolkit
This repository is based on RVinci from https://github.com/simonleonard/rvinci and 3D interaction cursors from https://github.com/aleeper/interaction_cursor_3d.

# How to run 
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
```
roslaunch rvinci rvinci_rviz.launch
```
To use MTM measurement toolkit:
```
rostopic pub /rvinci_measurement_MTM std_msgs/Bool "data: true"
```
To use PSM measurement toolkit:
```
rostopic pub /rvinci_measurement_MTM std_msgs/Bool "data: false"
```
