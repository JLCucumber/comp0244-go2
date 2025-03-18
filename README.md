# License

This project is licensed under the MIT License - see the LICENSE file for details.

# Author

- Hongbo Li (Code)
- Yiyang Jia (Code)
- Xinyun Mo (Testing)

# Contribution  (Time & Percentage)

- Hongbo Li (Task1(30h) + Task2(12.5h) + Task3 (12.5h)) => 55 Hours in total

  Implemented Edge Following algorithum adjusting Waypoint Follower and launch files, testing and fixing bugs in all three tasks, drafting of report and README file.  

- Yiyang Jia  (Task1(10h) + Task2(20h) + Task3 (20h)) => 50 Hours in total

  Implemented Bug0 and Bug1 algorithum and launch files, testing and fixing bugs in all three tasks, drafting of report and README file.

- Xinyun Mo  (Task1(10h) + Task2(10h) + Task3 (10h)) => 30 Hours in total

  Testing the pipeline, responsible for refining report.

# Environment Set-up

### BEFORE YOU START

***The following tutorial has only been tested on Ubuntu 22.04 system.***

## 1. Create Project

Create your workspace and enter directory.

```bash
mkdir /home/$USER/workspace && cd /home/$USER/workspace

```

Clone the repo or extract folder `comp0244-go2` in the workspace. 

```bash
git clone --recursive git@github.com:JLCucumber/comp0244-go2.git

```

## 2. Docker Setup

### 2.1 Install Dependencies

```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL <https://download.docker.com/linux/ubuntu/gpg> -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \\
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] <https://download.docker.com/linux/ubuntu> \\
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \\
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt-get install x11-xserver-utils
xhost +
```

### 2.2 Install Docker

The following are instructions of different versions of Docker based on CPU and GPU for you to choose, respectively. In order to better simulate, you can choose GPU version to install

### i. CPU Version

In the same terminal, download the docker images.

```bash
sudo docker pull jjiao/comp0244:unitree-go-ros2-humble
sudo docker tag jjiao/comp0244:unitree-go-ros2-humble comp0244:unitree-go-ros2-humble

```

In the same terminal, create the docker container:

```bash
sudo docker run -it -e DISPLAY -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/tmp/.docker.xauth \\
-v /home/$USER/workspace:/workspace \\
--network host \\
--name comp0244_team2 comp0244:unitree-go-ros2-humble /bin/bash

```

Exit the docker and start your docker environment

```bash
sudo docker container start comp0244_team2
sudo docker exec -it comp0244_team2 /bin/bash

```

### ii. GPU Version (Recomended)

Install `docker-compose`.

```bash
sudo apt install docker-compose

```

Before starting the docker, check if graphic card driver running properly.

```bash
nvidia-smi
```

Install `nvidia-container-toolkit` to support GPU in docker.

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L <https://nvidia.github.io/nvidia-docker/gpgkey> | sudo apt-key add -
curl -s -L <https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list> | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Start docker.

```bash
sudo USER=$(whoami) docker-compose -f comp0244-go2/docker/compose_gpu.yml up -d
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

## 3. Build Project

In the same terminal, build the package before running the code.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /workspace/comp0244-go2/install/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
cd /workspace/comp0244-go2/src/livox_ros_driver2 && ./build.sh humble
cd /workspace/comp0244-go2 && colcon build
source /workspace/comp0244-go2/install/setup.bash

```

Install additional dependencies (same terminal)

```bash
apt install ros-humble-tf-transform* -y

```

# Code Executing

> Restart the current container after the first initial container is complete
> 
> 
> ```bash
> exit
> sudo docker restart comp0244_nvidia_team2
> sudo docker exec -it comp0244_nvidia_team2 /bin/bash
> 
> ```
> 

## Task 1

1. Enter the container environment:

```bash
## CPU Version
sudo docker exec -it comp0244_team2 /bin/bash
## GPU Version
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

1. Start the launch file: `ros2 launch cw1_team_2 run_solution_task_1.launch.py`
    1. By default, it launches `world_1.world`. To specify a different preset environment or a custom one, please refer to the appendix.

## Task 2

(Terminal #1)

1. Enter the container environment:

```bash
## CPU Version
sudo docker exec -it comp0244_team2 /bin/bash
## GPU Version
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

1. Start the launch file: `ros2 launch cw1_team_2 run_solution_task_2.launch.py`
    1. By default, it launches `world_1.world`. To specify a different preset environment or a custom one, please refer to the appendix.

（Terminal #2）

1. Enter the container environment:

```bash
## CPU Version
sudo docker exec -it comp0244_team2 /bin/bash
## GPU Version
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

1. Pass in Target Waypoint：`ros2 param load /Bug0 src/cw1_team_2/cw1_team_2/config/robot_params_bug0.yaml`
2. The robot now should start moving

## Task 3

（Terminal #1）

1. Enter container environment

```bash
## CPU Version
sudo docker exec -it comp0244_team2 /bin/bash
## GPU Version
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

1. Start the launch file: `ros2 launch cw1_team_2 run_solution_task_3.launch.py`
    1. By default, it launches `world_1.world`. To specify a different preset environment or a custom one, please refer to the appendix.

（Terminal #2）

1. Enter container environment

```bash
## CPU Version
sudo docker exec -it comp0244_team2 /bin/bash
## GPU Version
sudo docker exec -it comp0244_nvidia_team2 /bin/bash
```

1. Pass in Target Waypoint：`ros2 param load /Bug0 src/cw1_team_2/cw1_team_2/config/robot_params_bug1.yaml`
2. The robot now should start moving

> Please note: We have observed significant odometry drift while the robot is moving, which may cause distortion in the determination of the start point and leave point for Bug1. Due to time constraints, we have not looked into the implementation of the `ChampOdometry` class in `unitree-go2-ros2` in detail.
> 

---

# Tasks Implementation

> Here, we will discuss the implementation ideas and details
> 

## Task 1

### **cw1_edge_follower (edge_follower)**

1. **Handling Sharp Turns**

- **Overshoot Compensation:** The robot moves past the end of the detected line before turning to ensure a smoother transition, controlled by `self.OVERSHOOT = 0.3` meters.
- **End-Line Lag:** At the end of the path, the robot continues moving in the original direction for one additional step to reduce abrupt turns.
- **Corner Waypoint Smoothing:** Waypoints near corners are adjusted using a weighted average to prevent sharp turns:
\text{current_waypoint} = 0.6 \times \text{current_waypoint} + 0.4 \times \text{self.last_waypoint}

2. **Suppressing the "Edge Jumping" Issue**

- A threshold of `self.POINT_THRESHOLD = 0.8` meters ensures stable edge detection.
- Edge points are updated only if both conditions are met:
    1. Distance to the last closest point is below the threshold:
    \text{distance_to_last_time_closest_point} < \text{self.POINT_THRESHOLD}
    2. Distance to the robot is less than the minimum recorded distance:
    \text{distance_to_robot} < \text{min_distance}

3. **Determining Counterclockwise Direction**

- **Replaced cross-product calculation with a manually set direction flag:**
    - The movement direction is now explicitly defined as `self.moving_forward = "counter-clockwise"`.
    - The previous cross-product method was unreliable due to incorrect vector selection, as it did not account for the robot's orientation. The manually assigned flag ensures correct navigation.

### 4. **Extending Waypoint Information**

- **Adding Direction Information:** Waypoints are expanded from (x,y) to (x,y,θ), where the angle is calculated as:
\text{edge_angle} = \text{atan2}(\text{edge_direction}[1], \text{edge_direction}[0])
    
    (x,y)(x, y)
    
    (x,y,θ)(x, y, \theta)
    
- **180° Rotation Adjustment:**\text{waypoint} = \left[\text{waypoint}[0], \text{waypoint}[1], \text{edge_angle} + \pi\right]
- **Visualization Improvement:** Waypoints are now displayed as arrows instead of spheres, making movement direction more intuitive.

**cw1_waypoint_follower (waypoint_follower)**

1. **Modifying `control_loop_callback(self)`**

The original motion logic (turn → move forward → turn) often caused the robot to get stuck when waypoints changed frequently, preventing smooth movement.

2. **New Motion Logic: Continuous Forward Movement with Dynamic Angular Adjustment**

Instead of stopping to turn, the robot now moves forward while adjusting angular velocity in real-time. Special handling is added for:

- **Moving Backward:** If the target is behind the robot with a similar orientation, it moves backward instead of turning in place.
- **Urgent Turn:** If a large turn is required (>0.4π) and the target is not too close, the robot performs a quick rotation while moving slightly forward.

3. **Standard Navigation & Final Alignment**

For regular movement, the robot maintains maximum velocity while dynamically adjusting its direction. Upon reaching the waypoint, it stops and fine-tunes its orientation before marking arrival

## Task 2

The core idea of the Bug0 algorithm is **to move in a straight line, follow the edge when encountering an obstacle, and continue towards the goal after avoiding the obstacle**. In this implementation, **`AdvancedEdgeFollowerNodes`** acts as a **child node** to **control and manage** the robot, primarily for edge detection and obstacle tracking. The following are the specific steps

1. **Child Node `edge_follower` Initialization and Subscription Management**
    - `BugPlanner` inherits from `Node` and creates `AdvancedEdgeFollowerNodes` as **a child node** to handle boundary detection.
    - It subscribes to **`Odometry` and `local_map_lines`** topics, and the message callbacks are handled by the `edge_follower` internal functions `odom_callback()` and `line_callback()`, rather than `BugPlanner` handling them directly.
2. **Data Acquisition and use `spin_once` to Call Child Node**
    - During the `move_to_goal()` and `update_data()` processes, `BugPlanner` needs to retrieve the latest sensor information.
    - Since `edge_follower` runs independently as a child node,`BugPlanner`  **actively triggers the child node to execute** through **`rclpy.spin_once(self.edge_follower)`**to receive the latest obstacle data.
3. **Path Planning and Obstacle Detection**
    - `move_to_goal()` calculates the direction to the goal and uses `is_obstacle_detected()` to check for obstacles.
    - By **calling the child node’s `transform_to_base_link()`**, the target coordinates are transformed to the robot’s coordinate frame for navigation.
    - If an obstacle is detected, `BugPlanner` pauses its own `timer` and calls `edge_follower` to continue avoiding the obstacle.
4. **Robot Control and Navigation**
    - **Once the obstacle is cleared, `timer` is reactivated, and the robot continues moving**.
    - Navigation commands are published using `Pose2D` via `waypoint_pub` to guide the robot’s movement.

## Task 3

The core idea of the Bug1 algorithm is **to move in a straight line, follow the edge when encountering an obstacle, and continue towards the goal after finding the optimal exit point.** In this implementation, **`AdvancedEdgeFollowerNodes`** is used as a **child node** to **control and manage the process**, primarily for edge detection and obstacle tracking. The following are the specific steps: 

**1. Initialization of Child Node `edge_follower` and Subscription Management**

- `BugPlanner` inherits from `Node` and creates **`AdvancedEdgeFollowerNodes` as a child node**, responsible for boundary detection.
- It subscribes to **`Odometry` and `local_map_lines`** topics, with message callbacks handled by `odom_callback()` and `line_callback()` inside `edge_follower`, rather than being processed directly by `BugPlanner`.

**2. Data Retrieval and `spin_once` for Child Node Execution**

- During `move_to_goal()` and `update_data()`, `BugPlanner` needs to obtain the latest sensor information.
- Since `edge_follower` operates **as an independent child node**, `BugPlanner` actively triggers its execution using **`rclpy.spin_once(self.edge_follower)`**, ensuring real-time obstacle information updates.

**3. Path Planning and Obstacle Detection**

- `move_to_goal()` calculates the robot’s direction toward the goal and checks for obstacles using `is_obstacle_detected()`.
- The goal coordinates are transformed into the robot’s coordinate frame via **a call to the child node's `transform_to_base_link()`**, enabling accurate navigation.
- If an obstacle is detected, `BugPlanner` suspends its timer with `timer.cancel()`, invokes `edge_follower` to handle obstacle avoidance, and records the **contact point `loop_start_point`** to determine when obstacle circumnavigation is complete.

**4. Recording the Optimal Exit Point During Obstacle Circumnavigation**

- **During obstacle circumnavigation**, `BugPlanner` continuously updates data using `rclpy.spin_once(self.edge_follower)`, identifying the closest position to the goal as `leaving_point`.
- **After completing a full loop around the obstacle**, if `BugPlanner` finds that `leaving_point` is closer to the goal than `loop_start_point`, it exits from `leaving_point`; otherwise, it continues following the obstacle boundary.

**5. Robot Control and Navigation**

- **Once the obstacle is cleared, `timer` is reactivated, allowing the robot to resume its path.**
- **Navigation commands are published using `Pose2D` via `waypoint_pub`**, directing the robot toward its target.

---

# Appendix

## 1. Gazebo Preset Scenes and Loading Custom Scenes

### Regarding the preset Gazebo scenes:

We have prepared several preset Gazebo environments to simplify testing. These scenes are stored in the `src/cw1_team_2/cw1_team_2/environment/` directory. Each scene has been configured with specific object placements and the initial positioning of the **Go2** robot to match different testing requirements.

When using `ros2 launch` to start a task, the default scene, `world_0`, will be loaded. However, you can customize the environment by specifying a different preset world in the command line. This flexibility allows you to test in various environments without modifying the underlying code.

For example, if you want to launch the third preset scene, you can use a command like:

```bash
 ros2 launch cw1_team_2 run_solution_task_1.launch.py gazebo_world:=world_3
```

> There is no need to recompile the scene! Simply run the following command to load your custom scene: ros2 launch {your_launch_file} gazebo_world:={your_world_name}
> 

### How to Load a Custom Gazebo Scene

1. Save your custom Gazebo scene as a `.world` file (e.g., `my_gazebo_world.world`).
2. Place the file in the `src/cw1_team_2/cw1_team_2/environment/` directory.
3. Rebuild the workspace: `cd /workspace/comp0244-go2/ && colcon build && source install/setup.bash`
4. Run the task with the custom world:: `ros2 launch cw1_team_2 run_solution_task_1.launch.py gazebo_world:=my_gazebo_world` (Adjust the specific task name and Gazebo world name based on your actual setup)

Now, you should see Gazebo launch with your custom scene.

## 2. Run Docker on VSCode.

1. Enter from VSCode (recommended):
Launch VSCode ⇒ Open the Docker extension page ⇒ Right-click on `comp0244_nvidia_team2` ⇒ Attach in VSCode.
    
    https://github.com/user-attachments/assets/b8355146-cca3-428e-9ff0-3d01c8cdc2e2
    

## 3. Passing the Target Waypoint Parameter

1. Path: `src/cw1_team_2/cw1_team_2/config`，which includes `robot_params_bug0.yaml` and `robot_params_bug1.yaml`。
2. Customizing new Target Waypoint parameters:
    - set the parameters according to the format in the `.yaml` files.
3. Waypoint Coordinate System: Ensure that the waypoint coordinates are defined in the correct coordinate system, typically the world or robot base frame, depending on your specific use case.
