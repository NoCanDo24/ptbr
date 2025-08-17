# Das Pass The Butter Robot Packet

In diesem Repository befindet sich das selbstprogrammierte ROS2 Jazzy Packet für den "Pass The Butter Robot". Es lassen sich alle launchfiles finden, wie auch die nodes, welche den Roboter betreiben.

# Voraussetzungen

Wichtige Packete, welche auf welche dieses Packet Aufbaut sind folgende:

#### - RTAB-map von [hier](https://github.com/introlab/rtabmap_ros/tree/ros2)
#### - DepthAI-ROS driver von [hier](https://docs.luxonis.com/software/ros/depthai-ros/build/)
#### - Nav3 von [hier](https://docs.nav2.org/getting_started/index.html#installation)

# Ausführen der Software

joints.launch.py für Motortreiber und Gelenkkontrolle

robot.launch.py für SLAM, URDF, Motortreiber und Gelenkkontrolle

nav2.launch.py für Nav2

Die Pose des Roboters, wie auch die Art der Steuerung wird durch parameter gesteuert:

manual_control von der motor_controller node, True für zugriff von Teleop_keyboard, False für Nav2 control

upright_state von der joint_controller node, True für aufrechte Haltung und Arme unten, False für beugung nach vorne und gestreckte Arme
