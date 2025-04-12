

### How to create python pkg 
```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```

### How to build all pkgs
```bash
cd ~/ros2_ws
# It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build
```

### How to build a specific pkg
```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

### how to run your node
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run pkg_name node_exec
```

