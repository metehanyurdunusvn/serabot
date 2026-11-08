# SeraBot live verification record (7 November 2026)

**Workspace:** `/home/samet/serabot` under WSL Ubuntu 26.04.
**Source boundary:** only new files within `src/serabot_vision/` changed during
vision navigation development; original Metehan-owned sources are untouched.
Verification code and evidence were subsequently committed and pushed to GitHub.

## Components

- OpenCV dual-range HSV red detection and blob width measurement.
- `VisionNavigator`: SEARCH / ALIGN / APPROACH / AVOID_TURN /
  AVOID_DRIVE / TARGET_STOP behavior.
- Odometry-anchored memory of the camera-estimated red goal.
- LiDAR collision corridor (chassis-aware lateral filtering), safe detour
  turns and short odometry-measured forward segments.
- Large centered red target + LiDAR-conditioned stopping, with stop latch.
- /scan /odom /camera freshness checks and stop on data timeout.
- Only one velocity command publisher (TwistStamped /cmd_vel).

## Tests

Command:
```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
source install/setup.bash
colcon test --packages-select serabot_vision serabot_behavior
colcon test-result --test-result-base build/serabot_vision --verbose
colcon test-result --test-result-base build/serabot_behavior --verbose
```

Latest results (8 November): `serabot_vision`: 22 tests, 0 failures,
0 errors, 0 skipped. `serabot_behavior`: 19 tests, 0 failures, 0 errors,
1 skipped. **Total: 41, 0 failures, 0 errors, 1 skipped.**

## Gazebo observations

1. The original first vision controller recognized red and approached, but
   stalled at a neighbouring pot (`obstacle_blocked`). This was a FAILURE
   for default-spawn navigation; no collision was recorded.
2. First detour version passed the first pot but lost red during detours and
   eventually patrolled away from the target. FAILURE, reproduced via odom
   and camera telemetry.
3. Added red-goal odometry memory, collision-corridor filtering, near-target
   alignment priority and a safe vision-verified standoff.
4. Short obstructed scenario, spawn x=-1.6, y=0.7, yaw=0:
   `avoid_turn -> avoid_drive -> align_target -> approach_target ->
   target_reached`. Final odom relative to spawn approximately
   (x=1.69m, y=0.11m, yaw=0.57rad); front laser 0.8-0.83m;
   red centered, linear/angular /cmd_vel=0.
5. **Default-spawn scenario**, x=-4.0, y=0.7, yaw=0:
   detected red, moved through first pot group, avoided further obstacles,
   remembered/reacquired red after losing it, and finally transitioned
   `approach_target -> target_reached`. Final odom relative to spawn
   approximately (x=4.08m, y=0.16m, yaw=0.57rad).
   Last observed front laser range approximately **0.82m**, target image
   width 165px with near-zero center offset and zero /cmd_vel in both axes.
   ROS /cmd_vel had one active publisher and one subscriber.

## Evidence files

- `red_plant_detection.png`: red blob marked in earlier close demo.
- `red_plant_reached.png`: close obstructed scenario after reaching red.
- `red_plant_reached_default.png`: default-spawn scenario after success.
- `normal_start_trajectory.csv`: periodic /odom, /cmd_vel, camera target
  and front scan measurements from the successful default-spawn test.

## Reproduce the successful default-spawn run

Terminal 1:
```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch serabot_gazebo sim.launch.py rviz:=false behavior:=false
```

Terminal 2:
```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 run serabot_vision red_plant_tracker --ros-args -p use_sim_time:=true
```

Note that the stop is more conservative than the minimum 0.65m threshold
in the successful run (approximately 0.82m observed). The result was
verified in this greenhouse, not guaranteed for every obstacle arrangement.
On 8 November a successful new default-spawn run was captured as
`SeraBot_Demo_Backup.mp4` with actual Gazebo top-down renders, camera images,
/scan and /odom overlays, and separate IMU evidence. The video was accelerated
for presentation; simulation sensor measurements were not altered. The final
front range in this separate recorded trial was around 0.48m; 0.82m belongs
to the separate 7 November baseline trial.

Additional 8 November evidence: `imu_motion_log.csv`, `imu_sensor_evidence.png`,
`topdown_start_2026_11_08.png`, `topdown_goal_2026_11_08.png`.

The latest speed tuning used approach=0.20m/s, detour=0.22m/s,
turn=0.60rad/s; obstacle and stop safety checks were retained.


## 8 November: two-mode navigation improvement (verified)

The previous reactive default-spawn demonstration required several turns
around multiple pot rows. We retained camera/LiDAR reactive avoidance as
a **separate test mode**, and updated the default demonstration to follow the
known clear aisle for 3.55m under LiDAR collision checking before switching
to proportional camera target alignment. This reduces repeated correction
and makes the main mission easier to demonstrate. The aisle leg is based
on a known map layout, NOT general SLAM or mapping.

### A. Reactive obstacle detour (separate starting pose)

* Spawn arguments: `x:=-1.6 y:=0.7 yaw:=0.0`
* Navigator: `-p use_sim_time:=true -p aisle_cruise_distance:=0.0`
* Observed sequence: `align_target -> approach_target -> avoid_turn
  -> avoid_drive -> align_target -> approach_target -> target_reached`
* Recorded final forward LiDAR range: about 0.805m
* Final camera red width: 166px; zero linear/angular commanded velocity
* Recorded native Gazebo screenshots and actual camera images:
  `obstacle_detour_2026_11_08.csv` and `obstacle_scenario_preview.jpg`

### B. Default-spawn known-aisle navigation

* Spawn: `x:=-4.0 y:=0.7 yaw:=0.0`
* Navigator default `aisle_cruise_distance=3.55m`
* Observed sequence: `aisle_cruise -> align_target -> approach_target
  -> target_reached` with no repeated avoidance turn loop.
* Recorded final forward LiDAR range: about 0.820m
* Final camera red width: 165px; zero linear/angular commanded velocity
* Actual telemetry: `aisle_cruise_2026_11_08.csv`, preview
  `aisle_scenario_preview.jpg`

### New final presentation backup

`SeraBot_Demo_Final_2_Scenarios.mp4` combines A and B as explicitly
different runs, 130.33s / 1280x720 MP4. The recorded Gazebo/ROS data are
real and unchanged, while video playback is accelerated about 1.8x
relative to the captured wall clock. The previous demo video is retained.

Final controller/test source is in the corresponding commit. The
sensor-reading and stop safety principles are unchanged. The two-mode
tests must not be presented as proof of general obstacle-free navigation
in arbitrary greenhouses.
