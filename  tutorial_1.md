# Установка ROS2 (версия eloquent) на Ubuntu LInux 18.04
https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html

Установка colcon:
```
sudo apt install python3-colcon-common-extensions
```

Создание workspace:
```
source /opt/ros/eloquent/setup.bash
mkdir -p ~/dev_ws/src
```

## Пример №1. Turtlesim (робот черепаха 2d)
https://docs.ros.org/en/eloquent/Tutorials/Turtlesim/Introducing-Turtlesim.html
```
cd ~/dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b eloquent-devel
cd ~/dev_ws/
rosdep install -i --from-path src --rosdistro eloquent -y
colcon build
. install/local_setup.bash
```

Запуск черепашки:

```
ros2 run turtlesim turtlesim_node
```

Управление черепашкой:
```
ros2 run turtlesim turtle_teleop_key
```

Изучение средств разработчика RQt
```
rqt
```

## Пример №2. Универсальный формат описания роботов (URDF). Робот R2D2.
https://docs.ros.org/en/eloquent/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html
```
git clone https://github.com/benbongalon/ros2-urdf-tutorial.git
```
change executable to node_executable
```
colcon build --symlink-install --packages-select urdf_tutorial
```
```
ros2 launch urdf_tutorial demo.launch.py

export LANG=en_US.UTF-8
rviz2 -d ~/dev_ws/install/urdf_tutorial/share/urdf_tutorial/r2d2.rviz
```