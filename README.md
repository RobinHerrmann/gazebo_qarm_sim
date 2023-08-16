# gazebo_qarm_sim
Package provides simulation of Quanser QArm in Gazebo.

---

Dieses Repository beinhaltet ein ROS2 Package für die Simulation des Quanser QArm Gelenkarmroboters in Gazebo. Der Roboterarm und seine Umgebung sind so modelliert, dass die Anwendung von Reinforcement Learning mithilfe des am OpenAI Gym Standard angelehnten [QArmEnv](https://github.com/deltawafer/gazebo_qarm_env) möglich ist.

---

## Prequisites
Zur Nutzung müssen Vorbereitungen getroffen werden. Die benötigte Software und Versionen sind in folgender Tabelle aufgelistet. Die Software kann den jeweiligen Anleitungen entsprechend installiert werden. Es wird empfohlen entsprechend [dieser](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) Reihenfolge vorzugehen.

Betriebssystem: Ubuntu 22.04

Software | Version
--- | ---
[ROS2](https://index.ros.org/doc/ros2/Installation) | Humble Hawksbill
[Gazebo](http://gazebosim.org/tutorials?cat=install) | 11
[ros2_control](https://control.ros.org/master/doc/getting_started/getting_started.html) | latest compatible
[gazebo_ros_pkgs](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) | latest compatible
[XACRO](http://wiki.ros.org/xacro) | latest compatible

---

## Installation

1. [ROS2 Workspace einrichten](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. "qarm_v1" Ordner in `/workspace/src` kopieren
3. `colcon build` in `/workspace` ausführen, um das Projekt zu bauen
4. `ros2 launch qarm_v1 display.launch.py` startet die Visualisierung in RVIZ zur Überprüfung
5. `ros2 launch qarm_v1 simulate.launch.py` startet die Siulation in Gazebo

--- 

## Zitieren

```
@software{gazebo_qarm_sim,
  author = {Herrmann, Robin},
  title = {{gazebo_qarm_sim}},
  url = {https://github.com/RobinHerrmann/gazebo_qarm_sim},
  version = {1.0},
  year = {2023}
}
```
