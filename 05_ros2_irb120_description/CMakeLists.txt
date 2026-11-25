cmake_minimum_required(VERSION 3.8)
project(irb120_description)

find_package(ament_cmake REQUIRED)

# Instala los directorios de datos en el share del paquete.
# Sin esto, package://irb120_description/... NO resuelve en RViz.
install(DIRECTORY 01_urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY 02_launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY 03_rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY 04_scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

ament_package()
