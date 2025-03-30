# License

This project is licensed under the MIT License - see the LICENSE file for details.

# Author

- Hongbo Li (Code)
- Yiyang Jia (Code)
- Xinyun Mo (Task1)

# Contribution  (Time & Percentage)

- Hongbo Li (Task1(30h) + Task2(12.5h) + Task3 (12.5h)) => 55 Hours in total

  Implemented Edge Following algorithum adjusting Waypoint Follower and launch files, testing and fixing bugs in all three tasks, drafting of report and README file.  

- Yiyang Jia  (Task1(10h) + Task2(20h) + Task3 (20h)) => 50 Hours in total

  Implemented Bug0 and Bug1 algorithum and launch files, testing and fixing bugs in all three tasks, drafting of report and README file.

- Xinyun Mo  (Task1(30h)) => 30 Hours in total

  ROS bag data extraction and analysis, comparing different algorithms (FSM, Neural Network, and Random Forest) for predicting the quadruped robot’s contact states.

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
git clone --recursive git@github.com:JLCucumber/comp0244-go2.git -b CW2

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

The following is instruction of Docker based on GPU for better simulation.


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
cd /workspace/comp0244-go2/unitree_ros2/cyclonedds_ws && colcon build --packages-select cyclonedds
colcon build
source /workspace/comp0244-go2/install/setup.bash
```



# Code Executing
Real Robot Setting Up
```bash
source /workspace/comp0244-go2/unitree_ros2/setup.sh
```


---

# Tasks Implementation

> Here, we will discuss the implementation ideas and details

## Task 1

The code implements a **Random Forest classifier** to predict contact states of a robot using **IMU (Inertial Measurement Unit) data** and **effort (motor torque) data**.  

### 1. Data Loading & Preprocessing  
- Reads **IMU, effort, and contact state data** from CSV files.  
- Merges IMU and effort data based on timestamps to create the feature dataset.  
- Aligns contact state data using **nearest neighbor matching** to ensure timestamp consistency.  

### 2. Feature Extraction & Model Training  
- The feature matrix (`X`) is obtained from IMU and effort data, while labels (`y`) correspond to contact states (`contact_1` to `contact_4`).  
- Missing values are handled by filling with zeros.  
- Trains a **Random Forest classifier** for each contact state.  

### 3. Prediction on Test Data  
- Loads new test data and aligns features with contact state timestamps.  
- Uses the trained models to predict contact states for the test dataset.  

### 4. Saving Predictions  
- Stores the predicted contact states in a CSV file (`predicted_contacts.csv`) for further analysis.  



# Appendix

## 1. Run Docker on VSCode.

1. Enter from VSCode (recommended):
Launch VSCode ⇒ Open the Docker extension page ⇒ Right-click on `comp0244_nvidia_team2` ⇒ Attach in VSCode.
    
    https://github.com/user-attachments/assets/b8355146-cca3-428e-9ff0-3d01c8cdc2e2
    