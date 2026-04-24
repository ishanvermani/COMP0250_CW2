# COMP0250 Coursework 2 — cw2_team_11

> **Disclaimer:** This README was drafted by Anthropic Claude under direction and review of the team.

---

## Authors

| Name  | GitHub / UCL ID |
|-------|-----------------|
| Ogulcan Gurelli | ucabure        |
| Ishan Vermani | rmapive            |
| Christos Toilos|      rmapcto         |

---

## Contribution Statement

| Task | Contributor(s) | Split | Approx. Hours |
|------|----------------|-------|---------------|
| Task 1 | Ogulcan, Chris, Ishan | 45% / 45% / 10% | 20 hrs total |
| Task 2 | Ishan, Chris | 90% / 10% | 20 hrs each |
| Task 3 | Ogulcan, Chris, Ishan | 33% / 33% / 33% | 10 hrs total |

---

## Dependencies

Before building or running, ensure the following are sourced / installed in your shell:

- **ROS 2 Humble** — `source /opt/ros/humble/setup.bash`
- **MoveIt!** — `ros-humble-moveit`, `ros-humble-moveit-core`, `ros-humble-moveit-ros-planning-interface`
- **PCL** — `ros-humble-pcl-ros`, `ros-humble-pcl-conversions`, `ros-humble-point-cloud-transport`
- **Gazebo ROS 2 Control** — `ros-humble-gazebo-ros2-control`
- **TF2** — `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs`, `ros-humble-tf2-sensor-msgs`

---

## Prep

To get started:

1) Clone or pull the updated surgical vision lab repo
https://github.com/surgical-vision/comp0250_s26_labs

2) navigate to the courseworks directory 

```bash
cd src/courseworks
```

3) delete `cw2_team_x`

```bash
rm -rf cw2_team_x
```

4) in `src/courseworks` run

```bash
git submodule add https://github.com/ishanvermani/COMP0250_CW2.git
```

This will add the cw2_team_11 file, nested in the COMP0250_CW2 folder. This does not impact the build or the code, as the ROS2 package is still installed at the same level as before. 

5) to add coursework 1, run

```bash
git submodule add https://github.com/oljen/comp0250_cw2_team11.git
```

## Build

```bash
cd ~/comp0250_S26_labs
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
---

## Run

### Launch the solution

```bash
ros2 launch cw2_team_11 run_solution.launch.py \
  use_gazebo_gui:=true use_rviz:=true \
  enable_realsense:=true enable_camera_processing:=false \
  control_mode:=effort
```

Useful launch flag overrides:

| Flag | Default | Description |
|------|---------|-------------|
| `use_gazebo_gui` | `true` | Show the Gazebo GUI |
| `use_rviz` | `true` | Show RViz — set `false` to suppress: `use_rviz:=false` |
| `enable_realsense` | `true` | Enable the depth camera |
| `control_mode` | `effort` | Robot controller mode |

### Trigger a task

In a separate terminal, run the following commands. 

```bash

cd ~/comp0250_s26_labs
source /opt/ros/humble/setup.bash
source install/setup.bash


# Task 1 — Pick and Place (single shape, position given)
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 1}"

# Task 2 — Shape Identification (camera-based shape classification)
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 2}"

# Task 3 — Autonomous Sort (detect and sort all shapes)
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 3}"
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Gazebo hangs or won't start | `killall gzserver gzclient` then relaunch |
| RViz not wanted | Add `use_rviz:=false` to the launch command |
| Adding this package to a new workspace | Add `tf2_ros` to `CMakeLists.txt` |
| Basket not detected correctly in a dark Gazebo environment | Dark lighting reduces RGB values seen by the camera. Adjust the `brown` entry in the `colors` array in `cw2_class.h` to match the true observed colours in your environment |
| Box centroid is off-centre (box partially outside camera FOV) | If a box is cut off at the edge of the camera frame, the computed cluster centroid will be shifted away from the true box centre. Increase the `0.3 m` coordinate-matching tolerance (Task 2) to compensate |

---

## Key Assumptions

1. All physical dimensions from the coursework spec are treated as fixed and hard-coded.
2. All baskets and boxes are resting on the platform — nothing is floating.
3. Shapes do not overlap or nest inside each other.
4. If multpile instances of the same shape exist, any one of them is an acceptable drop target.

---

## Key Hard-Coded Values

| Value | Purpose |
|-------|---------|
| `0.07 m` | Maximum bounding width of a cube in the world plane (accounts for up to 45° rotation) |
| `0.30` | Euclidean (L2-norm) colour acceptance threshold for RGB matching |
| `0.3 m` | Maximum tolerated error between Task 2 provided coordinates and cluster-predicted coordinates |

---

## Engineering Notes

### Task 1 — Pick and Place

**Core method:** `cw2::pick_and_place(Pose obj_pose, Point basket_loc)`  
Inputs are the cube centroid pose and the basket centre point. The method is also reused by Task 3.

#### Fixed Global Grasp Orientation
The Panda arm has 7 DOF. Using relative joint rotations caused unpredictable end-effector orientations, often resulting in the gripper striking cube corners rather than flat faces. A fixed world-frame quaternion `(x=0.9238, y=-0.3826, z=0.0, w=0.0)` — corresponding to Roll=180°, Pitch=0°, Yaw=−45° — is fed directly into the Cartesian planner. This locks the gripper to the world grid and forces the IK solver to bend the arm accordingly, ensuring the fingers align with the cube faces on every attempt.

#### Gripper Width / Gazebo Tolerance
Commanding the gripper to close beyond the physical cube width caused the trajectory controller to keep expecting movement after the fingers had stopped. Once the position error exceeded the 0.03 m safety tolerance, the controller aborted and dropped the cube. The `strong_grip` function sets `panda_finger_joint1` to exactly `0.020 m`, creating a 0.040 m gap that matches the cube width precisely. The trajectory completes just as the fingers contact the rigid body, generating friction without triggering an abort.

#### Z-Height Floor Clearance
Descending to the exact cube centroid sometimes caused fingertips to scrape the floor, spiking friction and stalling the arm. A `+0.005 m` offset is added to the grasp height calculation, keeping fingertips safely clear of the collision floor while still achieving a secure grip.

#### Motion Planning Strategy
- **Cartesian waypoints** are used for linear descents and ascents to prevent unpredictable arcing near the object.  
- **Joint-space planning** is used for large transit moves between pick and place positions.

---

### Task 2 — Colour Identification

*See inline code comments for method-level details.*  
The camera pipeline moves the arm to a bird's-eye position, captures a point cloud from `/r200/` and uses passthrough filtering, plane segmentation, and Euclidean clustering to isolate individual baskets. Each cluster centroid is transformed to the world frame and matched to the nearest provided basket location (within the 0.3 m tolerance). The colour is determined by comparing the mean RGB of the cluster against the three reference colours using the 0.35 Euclidean threshold. Basket locations with no associated basket are returned as `"none"`.

---

### Task 3 — Autonomous Sort

Task 3 reuses the same PCL pipeline from Task 2 to detect both boxes and baskets, classifying each by colour. Boxes are paired with their matching-colour basket and the `pick_and_place` method is called iteratively. If no matching basket exists for a box colour, that box is skipped. If multiple baskets of a color exist, only one is used. The pipeline handles variable numbers of objects gracefully.

---

## Package Structure

```
cw2_team_11/
├── config/
│   └── pcl_params.yaml
├── include/
│   └── cw2_class.h
├── launch/
│   ├── run_solution.launch
│   └── run_solution.launch.py
├── rviz/
│   └── cw2_pcl.rviz
├── src/
│   ├── cw2_class.cpp
│   └── cw2_node.cpp
├── srv/
│   └── Example.srv
├── CMakeLists.txt
├── package.xml
└── README.md
```
