# Environment Set-up

1. 创建并进入文件夹 `mkdir workspace && cd workspace`
2. `git clone --recursive [git@github.com](mailto:git@github.com):JLCucumber/comp0244-go2.git`
3. (可选) 为获得更好的仿真效果, 需要使用 docker-compose 构建基于GPU 的环境
    1. 安装 docker-compose
    2. 进入`comp0244-go2/docker/` , 修改`compose_gpu.yml`
        
        ```yaml
            volumes:
              # This mounts the X11 socket into the container so that GUIs can be displayed
              - /tmp/.X11-unix:/tmp/.X11-unix:rw
              - /dev/input:/dev/input
              - /dev/bus/usb:/dev/bus/usb:rw
              - /home/$USER:/workspace:rw  #  home/user 改成你自己的路径 (git clone的位置)
        ```
        
    3. 启动容器, 这回构建一个名字为 `comp0244_ros_nvidia` 的 docker container
        
        ```bash
        # -d 让容器在后台运行
        docker-compose up -d  
        ```
        
    4. 进入容器: 
        1. 从 VScode 进入(推荐): 
        启动VSCode ⇒ 打开 Docker 拓展页面 ⇒  右键`comp0244_ros_nvidia` ⇒ attach in VScode 
        2. 命令行进入: `docker exec -it comp0244_ros_nvidia bash` 
        3. 不要忘记 `xhost +` 启用可视化
4. 最终效果
     ![image](https://github.com/user-attachments/assets/b8355146-cca3-428e-9ff0-3d01c8cdc2e2)

# Code Executing

> 以下活动请在 docker 中运行!
> 

## 2D Environment

1.  `cd comp0244-go2/src/cw1_team_2/2D_environment`
2. 可能要求安装 `shapely` , apt install 即可
3. `python3 main.py` 

![image](https://github.com/user-attachments/assets/5f078a7e-b765-4001-bc40-fb1ce0f55fc0)


## Task 1

> Update: Now you can simply run `ros2 launch cw1_team_2 environment_set_up.launch.py` to launch RViz and Gazebo
> To launch specific tasks, you can include `environment_set_up.launch.py` into your launch file

1. 回到 `/workspace/comp0244-go2` 
2. 每次启动终端自动加载humble环境变量  `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` 
3. 编译 ros2 环境
    1. 这里要先进入 `cd comp0244-go2/src/livox_ros_driver2 && ./build.sh humble`
    2. 然后 `cd ../..`  , `colcon build` , `source install/setup.bash` 
4. **(terminal #1)** `ros2 launch robot_launch_neo.launch.py`
    
    > `robot_launch_neo` 加载的 gazebo 环境我做了一点小改动, 去掉了一个长方体, 让绕柱更好观察
    > 
    1. 等待大概10 sec, 你应该能看到 rviz & gazebo 弹出
    2. 把 rviz 的配置改成如下. 改完以后保存到一个docker中的位置(我的是 `/workspace/comp0244-go2/src/cw1_team_2/env_config/rviz_config.rviz` )
![image](https://github.com/user-attachments/assets/53652342-5367-4d28-930a-11b0884ebfc1)

5. **(terminal #2)** `ros2 run cw1_team_2 cw1_edge_follower`
    1. 你应该能看到机器人开始运动.
