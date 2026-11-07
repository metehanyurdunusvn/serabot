# SeraBot live verification record (7 November 2026)

**Workspace:** `/home/samet/serabot` under WSL Ubuntu 26.04.
**Source boundary:** only new files within `src/serabot_vision/` changed during
vision navigation development; original Metehan-owned sources are untouched.
Project root remains locally ahead of GitHub by three earlier commits.
No vision commit or push was made during this verification.

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

Result: `serabot_vision`: 20 tests, 0 failures, 0 errors, 0 skipped.
`serabot_behavior`: 19 tests, 0 failures, 0 errors, 1 skipped.
**Total: 39, 0 errors, 0 failures, 1 skipped.**

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
No complete Gazebo-window screenshot or video was recorded.
