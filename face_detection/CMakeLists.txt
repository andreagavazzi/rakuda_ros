cmake_minimum_required(VERSION 3.5)
project(face_detection)

# Trova le dipendenze ROS 2 necessarie
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Installa gli script Python (setup.py lo gestisce)
ament_python_install_package(${PROJECT_NAME})

# Installa i file di lancio o altri script se li aggiungi in futuro
install(
  PROGRAMS
    ${PROJECT_NAME}/face_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
