# ROS 2 Drone MPC Project

A complete **ROS 2 + Python** project that controls the **3D displacement of a drone** using **Model Predictive Control (MPC)**.

This repository is designed as a **clean GitHub portfolio project**:
- fully commented code,
- simple ROS 2 package structure,
- educational simulator included,
- MPC node separated from the physics,
- easy to extend toward PX4 / MAVROS / real drone integration.

## What this project does

The project simulates a drone as a point-mass moving in 3D:
- state: position and velocity in `x, y, z`,
- input: acceleration command in `x, y, z`,
- control objective: track a desired position trajectory.

The architecture is:

1. **`reference_node.py`** publishes the desired trajectory.
2. **`mpc_controller_node.py`** reads the current state and computes the optimal acceleration with MPC.
3. **`drone_simulator_node.py`** simulates the drone dynamics.
4. **`plot_results.py`** optionally plots the logged results.

## Why this is a good project for GitHub

It shows several important skills in one repository:
- ROS 2 node development in Python,
- control engineering,
- MPC formulation,
- numerical optimization with `cvxpy`,
- simulation and evaluation,
- good project structure and documentation.

## ROS 2 target

This project is written for **ROS 2 Humble or Jazzy**.

Humble is still supported until **May 2027**, and Jazzy is supported until **May 2029** according to the ROS 2 distributions page. Python ROS 2 packages use the standard `ament_python` structure, and launch files should live in the package `launch/` directory and be exported through `setup.py`. citeturn308654view0turn308654view1turn308654view2

## Workspace structure

```text
ros2_drone_mpc_project/
├── README.md
├── requirements.txt
└── src/
    └── drone_mpc_ros/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── drone_mpc_ros
        ├── launch/
        │   └── drone_mpc_sim_launch.py
        ├── config/
        │   └── mpc_params.yaml
        └── drone_mpc_ros/
            ├── __init__.py
            ├── mpc_core.py
            ├── utils.py
            ├── reference_node.py
            ├── drone_simulator_node.py
            ├── mpc_controller_node.py
            └── plot_results.py
```

## Topics used

### Published topics
- `/drone/reference` (`geometry_msgs/PoseStamped`) : desired position
- `/drone/odom` (`nav_msgs/Odometry`) : simulated drone state
- `/drone/cmd_accel` (`geometry_msgs/AccelStamped`) : acceleration command from MPC

### Logged CSV
A CSV file is stored automatically in:

```bash
logs/drone_mpc_log.csv
```

It contains:
- time,
- reference position,
- measured position,
- measured velocity,
- commanded acceleration.

## Mathematical model

We use a simplified translational model:

\[
\dot{p} = v, \quad \dot{v} = u
\]

with:
- \(p = [x, y, z]^T\)
- \(v = [v_x, v_y, v_z]^T\)
- \(u = [a_x, a_y, a_z]^T\)

For one axis, the discrete-time model is:

\[
x_{k+1} = A x_k + B u_k
\]

with:

\[
x_k = \begin{bmatrix} p_k \\ v_k \end{bmatrix},
\quad
A = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix},
\quad
B = \begin{bmatrix} \frac{T_s^2}{2} \\ T_s \end{bmatrix}
\]

The full 3D model is obtained by stacking the three axes.

## MPC problem

At each control step, we solve:

\[
\min_{u_0,\dots,u_{N-1}} \sum_{k=0}^{N-1} (x_k-x_k^{ref})^T Q (x_k-x_k^{ref}) + u_k^T R u_k + (x_N-x_N^{ref})^T P (x_N-x_N^{ref})
\]

subject to:
- system dynamics,
- acceleration bounds,
- velocity bounds.

This is a **quadratic program (QP)** solved with **CVXPY**.

## Installation

### 1) Install ROS 2
Use a standard ROS 2 installation for **Humble** or **Jazzy**.

### 2) Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Copy this repository inside `~/ros2_ws/src/`.

### 3) Install Python dependencies

```bash
cd ~/ros2_ws/src/ros2_drone_mpc_project
python3 -m pip install -r requirements.txt
```

### 4) Install ROS dependencies

```bash
cd ~/ros2_ws
rosdep install -y --from-paths src --ignore-src
```

### 5) Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

For Jazzy, replace the source line with:

```bash
source /opt/ros/jazzy/setup.bash
```

## Run the simulation

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_mpc_ros drone_mpc_sim_launch.py
```

You should see logs printed by the simulator and controller.

## Plot the results after a run

```bash
cd ~/ros2_ws/src/ros2_drone_mpc_project
python3 src/drone_mpc_ros/drone_mpc_ros/plot_results.py --csv logs/drone_mpc_log.csv
```

## Nodes description

### `reference_node.py`
Publishes a smooth 3D reference trajectory.

Current default trajectory:
- move to `(1.0, 1.0, 1.0)`,
- then make a small horizontal circle,
- keep altitude close to `1.0 m`.

### `drone_simulator_node.py`
Implements a simple translational drone model.
It integrates acceleration into velocity and position.

This is **not a full quadrotor attitude model**.
It is deliberately simplified so the MPC logic stays readable.

### `mpc_controller_node.py`
Reads:
- current odometry,
- current reference,

and solves the finite-horizon MPC problem to publish the optimal acceleration.

### `mpc_core.py`
Contains the reusable MPC class:
- discrete model construction,
- QP setup,
- constraints,
- solve method.

## Adapting this project to a real drone

This project is educational and simulation-first.
To connect it to a real drone stack, you would typically:
- replace `drone_simulator_node.py` with real odometry feedback,
- map `/drone/cmd_accel` to your flight stack interface,
- add attitude control and thrust allocation,
- use safety layers and state estimation.

Typical real integrations are done through:
- PX4 + ROS 2 bridge,
- MAVROS / MAVSDK pipelines,
- onboard companion computer nodes.

## Possible improvements

Good ideas for extending the project:
- add disturbance rejection,
- add integral action,
- track a full time-varying reference over the horizon,
- replace the point-mass model by a full quadrotor model,
- add visualization in RViz,
- add a Gazebo or Ignition simulation,
- compare MPC vs PID.

## License

MIT
