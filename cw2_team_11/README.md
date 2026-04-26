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
| `0.30` | Euclidean (L2-norm) colour acceptance threshold for RGB matching |
| `0.100 m` | Threshold to determine if a found shape is duplicate or not. A shape with a centroid within this threshold from another centroid is ignored |
| `0.020m` | Inclusion radius from centroid that is used to define whether a shape is a cross or a nought. If points are in this radius, it is a cross |

---

## Engineering Notes

### Task 1, 3 — Pick and Place

**Core method:** `cw2::pick_and_place(Pose obj_pose, Point basket_loc)`  
Inputs are the cube centroid pose and the basket centre point. The method is also reused by Task 3.

#### For Task 1 and 3 and the shared pick-and-place function, we spent a lot of time figuring out what worked and what didn't, and the final approach reflects quite a few lessons learned. One of the first things we settled on was using Cartesian path planning for any vertical movement, going down to grab a shape, lifting it back up, and lowering it into the basket. This keeps the gripper moving in a straight line, which matters because even a slight arc can clip the edge of a shape or drag it across the table. For the bigger horizontal movements like travelling to the basket, we use regular joint-space planning since the exact path doesn't matter as much as just getting there efficiently. We also built in a simple fail-safe throughout the whole sequence, if any individual move fails, the gripper opens and the arm goes back to home. This stops the robot from getting stuck mid-air holding a shape with no plan for what to do next.
The detection side was probably the trickiest part. Because the depth camera is mounted on the robot's wrist, it can only see what's directly below the end effector, so we always move above the shape first before trying to detect anything. From there we run the point cloud through our pipeline — segment out the ground plane, cluster what's left, and classify each cluster using eigenvalue analysis and oriented bounding boxes. This tells us whether we're looking at a nought or a cross, what size it is, and what angle it's sitting at. That information drives everything else, the grasp offset from the shape's centre scales with size so the fingers always land on a grippable part of the wall or arm, and for noughts we rotate the gripper to match the shape's orientation so the fingers straddle the wall cleanly. Crosses don't need this rotation since their arms are symmetric.
One issue that took us a while to track down was the gripper itself. At slow speeds, the controller would time out before the fingers fully closed, which meant only one finger would move and the shape would just get pushed sideways instead of gripped. Setting both finger joints explicitly at full speed fixed this completely. We also use the exact coordinates from the service request for the shape's position rather than the point-cloud centroid, since the camera-derived position can shift a bit depending on viewing angle, but we still rely on the point cloud for size and yaw detection, since those aren't provided by the service.


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

### Task 1, 2, 3 — Shape and Colour Identification

The camera pipeline moves the arm to a bird's-eye position, captures a point cloud from `/r200/` and uses outlier removal, plane segmentation, and Euclidean clustering to identify shapes. Passthrough filtering in x and y was not used as the camera constantly moved to different positions. Instead, a z passthrough was used to remove clusters below the plane that were included in processing.

#### Principal Component Analysis (PCA) for Shape Classification
The PCL Moment of Inertia Estimator was used to obtain eigenvalues, eigenvectors, and an object-oriented bounding box (OBB) of each cluster. As our shapes had equal side dimensions, we knew we could exclude non square shapes. Therefore, if the major and middle eigenvalues did not match, we could safely exclude the shape, regardless of shape orientation. 

To identify whether a shape was a cross or a nought, the number of points within a threshold of 20mm was sampled. Crosses had central points, whereas noughts contained empty space. The number of observed points within this radius was used to define the shape. 

To define the size, two strategies were used. Crosses were classified using the OBB, which produces a box around the shape in the shape's orientation. The width of the OBB helped identify if the cross had cell size of 20, 30, or 40mm. 

As noughts have an added axis of symmetry across the diagonal, PCA would often return eigenvectors along the diagonal instead of edge to edge. The inconsistent behaviour of PCA made it difficult to use the OBB method to determine shape size, as a diagonal 30mm nought has the similar bounding box (212mm) as a aligned 40mm nought (200mm). Instead, the average distance of all points in a cluster was taken, and several thresholds were empirically determined to classify the shape cell size.

The major eigenvector was used to obtain the yaw of a cross (as the z axis is always the minor eigenvector). The yaw was adjusted to fit within 0 and 90 degrees, as the crosses and noughts are symmetrical in 2 axes. For noughts, the closest point to the centroid was found; this was assumed to be a point on the inner edge. The vector between the centroid and the point was used to define the shape's yaw.

The shape type, cell size, centroid location, and yaw serve as information for the pick and place functions in tasks 1 and 3. 

#### Colour Identification

The colour of a cluster was used for identifing obstacles and the basket in task 3. The average RGB of all points in the cluster were taken, from which the nearest defined colour was found.

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
