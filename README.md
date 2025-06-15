# NishCorp-Tech Wheelchair ROS 2 Package

A ROS 2 package for robotic wheelchair control, simulation, and visualization, developed by NishCorp-Tech. This package provides nodes and launch files for controlling, monitoring, and visualizing a robotic wheelchair platform in both RViz and Gazebo Ignition.

---

## Features

- Complete ROS 2 control system for wheelchair operation
- Position control for seat rotation, arms, and footrests
- Velocity control for all four wheels
- Visualization in RViz
- Full physics simulation in Gazebo Ignition
- Example launch files for quick startup
- Modular and extensible design

---

## Requirements

- ROS 2 Humble Hawksbill
- Python 3.8+
- Colcon build system
- RViz2
- Gazebo Ignition
- ros2_control and ros2_controllers
- ros_ign_gazebo

---

## Installation

1. **Clone the repository into your ROS 2 workspace:**

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/Siddhant-nishcorptech/NishCorp-Tech.git
    ```

2. **Install dependencies:**

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build the package:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select wheelchair_ros
    ```

4. **Source your workspace:**

    ```bash
    source install/setup.bash
    ```

---

## Usage

### Running the RViz Visualization

To launch the display (visualization) for the wheelchair in RViz:

```bash
ros2 launch wheelchair display.launch.py
```

### Running the Gazebo Simulation

To launch the wheelchair in Gazebo Ignition with full physics simulation:

```bash
ros2 launch wheelchair wheelchair_gazebo.launch.py
```

This will start Gazebo Ignition with the wheelchair model and all necessary controllers.

### Running the Teleop Controller

To control the wheelchair using keyboard inputs, follow these steps:

1. Launch the Gazebo simulation:
```bash
ros2 launch wheelchair wheelchair_gazebo.launch.py
```

2. Launch the wheelchair control:
```bash
ros2 launch wheelchair wheelchair_control.launch.py
```

3. Run the teleop controller:
```bash
ros2 run wheelchair teleop_controller.py
```

---

## Example Output

### RViz Visualization
![Wheelchair Visualization Example](wheelchair_ros/docs/wheelchair_rviz_example.png)

### Gazebo Simulation
![Wheelchair Gazebo Simulation](wheelchair_ros/docs/gazebo_1.gif)

### Teleop Control
![Wheelchair Teleop Control](wheelchair_ros/docs/teleop_control.gif)

---

## Controllers

The package includes two main controller groups:

### Position Controllers
- **Seat Rotation (j1)**: Controls the seat's rotation
- **Arms (j2, j3)**: Controls both wheelchair arms
- **Footrests (j4, j5)**: Controls both footrests

### Velocity Controllers
- **Wheel Control (j6, j7, j8, j9)**: Controls all four wheels independently

---

## Launch Files

- **display.launch.py**: Launches RViz visualization
- **wheelchair.launch.py**: Core wheelchair control and state publishing
- **wheelchair_gazebo.launch.py**: Launches Gazebo simulation with physics

---

## Configuration

The package uses YAML configuration files located in the `config` directory:
- **wheelchair.yaml**: Controller configurations
- **urdf.rviz**: RViz visualization settings
