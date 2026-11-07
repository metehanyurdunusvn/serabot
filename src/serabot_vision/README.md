# SeraBot red plant navigation

Sensor-driven ROS 2 behavior that combines camera-based red-plant detection,
LiDAR clearance checks and odometry-measured avoidance manoeuvres. This
package is independent of the original SeraBot source files.

## Build and test (WSL Ubuntu 26.04, ROS 2 Lyrical)

```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to serabot_vision
source install/setup.bash
colcon test --packages-select serabot_behavior serabot_vision
colcon test-result --verbose
```

Last verified on 7 November 2026: **39 tests; 0 errors, 0 failures,
1 skipped** (20 in serabot_vision, 19 in serabot_behavior).

## Verified default-spawn demo

Terminal 1 (starts Gazebo, robot, ROS bridge; default x=-4.0, y=0.7):

```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch serabot_gazebo sim.launch.py rviz:=false behavior:=false
```

Terminal 2 (separate controller, only one /cmd_vel publisher):

```bash
cd ~/serabot
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 run serabot_vision red_plant_tracker --ros-args -p use_sim_time:=true
```

RViz can be enabled by omitting `rviz:=false` if a graphical view is needed.
NEVER run the existing `obstacle_avoider`, teleop or any other velocity
publisher at the same time as this node.

Look for logged states:
`align_target` / `approach_target` / `avoid_turn` /
`avoid_drive` / `target_reached`.

Verify the final velocity and single publisher:
```bash
ros2 topic echo /cmd_vel --once
ros2 topic info /cmd_vel
```

## Algorithm

1. Detect saturated red via both HSV hue bands (0..7, 170..179) and use the
   largest plausible image blob. Reject brown pots using saturation thresholds.
2. Center the camera on red and drive cautiously using LiDAR.
3. Reject obstacles in an inflated body-width forward corridor. On blockage,
   choose a free side, turn in place and drive a measured detour distance
   based on /odom; repeat if blocked again.
4. During detour, remember the approximate red position using its camera
   bearing and apparent width (simulated red sphere diameter 0.36m) and
   odometry. Turn back toward this bearing even if red temporarily leaves
   the field of view. This is a simple reactive method, not full mapping.
5. As red approaches, prioritize final camera alignment and stop at a safe
   standoff. The standard hard stop is 0.65m front LiDAR range if a large red
   target is centered. For a very large, centered red target, an earlier
   conservative stop is permitted at a front range no greater than 0.95m.
   Both stop states latch until the node is restarted.
6. Stale /scan, /camera/image_raw or /odom data causes zero velocity.

## Real Gazebo checks (7 November 2026)

- From x=-1.6, y=0.7: bypassed a pot, reacquired the red goal and reached
  `target_reached`; final /cmd_vel was all zeros.
- From the **default spawn (-4.0, 0.7)**: navigated past plants, reacquired
  red after detours and reached `target_reached`.
- Default-spawn final observed front LiDAR distance: approximately **0.82m**,
  camera red offset around zero and zero linear/angular velocity.
- ROS /cmd_vel publisher count was **1** while the tracker was running.
- Camera evidence: `docs/evidence/red_plant_reached_default.png`.
  Trajectory and test notes: `docs/evidence/verification_notes.md`.

The diseased plant is the *existing* red sphere at approximately x=1, y=1.4
in the greenhouse. HSV red indicates a simulation color marker only: this
does NOT classify real plant disease. The verified example is not a claim
of reliability for arbitrary layouts, spawn points or lighting.

## Troubleshooting

- If the robot will not move, check `ros2 topic list` for /scan,
  /camera/image_raw, /odom and /clock.
- If two nodes publish /cmd_vel, stop the competing controller.
- If the robot cannot leave a new obstacle configuration, use the original
  `behavior:=true` demo to verify baseline LiDAR avoidance separately.
