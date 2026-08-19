# slam_bot

A ROS 2 package with a 2D occupancy-grid SLAM stack written from scratch, no external mapping or scan-matching libraries. Everything -- ray casting, ICP scan matching, and pose graph optimisation -- is implemented directly in Python using only numpy/scipy for linear algebra.

The package ships four independent mapper nodes that build a map of the same simulated environment (Gazebo) using progressively better localisation, so you can directly compare how each one behaves and drifts:

1. **Ground truth mapper** -- maps using the exact pose broadcast by the simulator.
2. **Raw odometry mapper** -- maps using only wheel odometry, with an identity `map -> odom` transform (no correction at all). This is the baseline that shows raw drift.
3. **ICP scan-matching mapper** -- corrects the odometry pose frame-to-frame using a custom point-to-point ICP (SVD-based rigid alignment) on consecutive lidar scans, and publishes a corrected `map -> odom` transform.
4. **Pose graph optimisation (PGO) mapper** -- builds a pose graph from the ICP-derived relative poses, adds loop-closure edges when the robot revisits a previously mapped area, and periodically optimises the whole graph (sparse Cholesky solve of the linearised system) to correct accumulated drift across the full trajectory and re-stamp the map.

All four run side by side in their own ROS namespace against the same simulated robot and lidar, each with its own RViz view, so the drift/correction behaviour of each approach is directly visible at the same time.

## Demo

![demo](docs/demo.gif)

The gif shows all four mappers running together. The raw odometry map (bottom left) drifts and never corrects. The ground truth map (top left) stays clean throughout. The ICP map (top right) is corrected scan-to-scan but can still accumulate drift over time. The pose graph optimisation panel is the one to watch (bottom right) -- you can see the map snap back into alignment when a loop closure is detected and the graph is optimised.

## How the mapping works

All four nodes share the same core occupancy grid mapping approach:

- A log-odds occupancy grid (400 x 500 cells at 0.025 m resolution by default) is maintained per mapper.
- Each lidar scan is converted from polar to cartesian and transformed into the map frame using the current pose estimate.
- Free space and occupied cells along each beam are updated using a self-written Bresenham line-drawing rasterisation, with separate log-odds increments for free (`L_free`) and occupied (`L_occ`) cells, clamped between `L_min` and `L_max`.
- The grid is converted to probabilities and published on `/map` (remapped per namespace) at a fixed timer rate.

Where the nodes differ is purely in how the robot pose used for each scan insertion is obtained:

- **`mapper_using_ground_truth.py`** reads the simulator's ground-truth pose topic directly and uses it as-is.
- **`mapper_using_raw_odom.py`** reads `/odom` directly and publishes an identity `map -> odom` transform, so any wheel-odometry drift shows up unfiltered in the map.
- **`mapper_using_odom_with_icp.py`** looks up the current `map -> base_link` pose via tf, and refines it every frame by running ICP between the current and previous lidar scans (nearest-neighbour correspondence search, SVD-based rotation estimation, outlier rejection via a max correspondence distance, convergence/stagnation checks). The resulting correction is folded back into a published `map -> odom` transform.
- **`mapper_with_pgo.py`** extends the ICP approach: every accepted ICP alignment becomes a sequential edge in a pose graph, keyframe scans are stored, and once enough nodes exist the node checks earlier nodes for proximity to detect loop closures. When one is found, a loop-closure edge is added and the graph is optimised (Gauss-Newton style least squares over the pose graph, sparse Cholesky solve), after which the map is fully re-rasterised from the corrected trajectory and the `map -> odom` transform is updated to match.

### `tf_merge_relay.py`

Since all four SLAM stacks run in parallel in their own namespace but publish against a shared simulated robot, each mapper publishes its own `map -> odom` correction on its own topic instead of the global `/tf`. `tf_merge_relay.py` is a small relay node that subscribes to a configurable list of input `TFMessage` topics (typically the global `/tf` plus that namespace's own tf output topic) and republishes them merged onto a single per-namespace `.../tf_merged` topic, which each namespace's RViz instance is pointed at. This is what lets four independent tf trees be visualised side by side without them clobbering each other.

## Package layout

```
slam_bot/
  slam_bot/
    mapper_using_ground_truth.py
    mapper_using_raw_odom.py
    mapper_using_odom_with_icp.py
    mapper_with_pgo.py
    tf_merge_relay.py
  launch/
    mapper_using_ground_truth.launch.py
    mapper_using_raw_odom.launch.py
    mapper_using_odom_with_icp.launch.py
    mapper_with_pgo.launch.py
    mappers_combined.launch.py
  urdf/
    slam_bot_mapping.urdf
  worlds/
    slam_bot_world.sdf
  config/
    joystick.yaml
  rviz/
    slam_bot_ground_truth.rviz
    slam_bot_raw_odom.rviz
    slam_bot_icp.rviz
    slam_bot_pgo.rviz
  models/
  docs/
    demo.gif
```

Each individual `*.launch.py` brings up the simulator, spawns the robot, and runs just that one mapper with its own RViz config. `mappers_combined.launch.py` brings up the simulator once and launches all four mappers, their tf relays, and their RViz windows together, each in its own namespace (`ground_truth`, `raw_odom`, `icp`, `pgo`).

## Requirements

- ROS 2 (tested with an `ament_python` package layout)
- Gazebo / `ros_gz_sim` and `ros_gz_bridge`
- `rviz2`
- `teleop_twist_joy` and `joy` (for joystick teleoperation of the robot)
- Python: `numpy`, `scipy`, `tf_transformations`

## Building

From your workspace root:

```bash
colcon build --packages-select slam_bot
source install/setup.bash
```

## Gazebo resource path

The world, robot model, and any referenced meshes live inside this package's `models` directory. Gazebo needs to be told where to find them before launching, otherwise spawning the robot or loading the world will fail. Export the resource path (in addition to sourcing your workspace) before running any launch file:

```bash
export GZ_SIM_RESOURCE_PATH=/<local path to slam_bot>/models

```
example:
```bash
export GZ_SIM_RESOURCE_PATH=/home/user/ros_ws/src/slam_bot/models

```

Add this to your `.bashrc` (or a workspace setup script) if you don't want to export it every time.

## Running

Run everything (all four mappers + RViz windows) at once:

```bash
ros2 launch slam_bot mappers_combined.launch.py
```

Or run a single mapper on its own:

```bash
ros2 launch slam_bot mapper_using_ground_truth.launch.py
ros2 launch slam_bot mapper_using_raw_odom.launch.py
ros2 launch slam_bot mapper_using_odom_with_icp.launch.py
ros2 launch slam_bot mapper_with_pgo.launch.py
```

Each launch file starts Gazebo, spawns the robot, brings up `robot_state_publisher`, the `ros_gz_bridge` (clock, cmd_vel, odom, scan, imu, tf), and a joystick teleop node so the robot can be driven around manually while mapping.

## Future work

- Integrating a navigation stack (planning and autonomous exploration) on top of the PGO map.
- Adding a vision-based front end (camera-based feature tracking / visual odometry or loop closure) alongside the existing lidar pipeline.

## License

MIT License

## Maintainer

karburettor (whykarthikwhy@gmail.com)