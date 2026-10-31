cmake_minimum_required(VERSION 3.8)
project(limo_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/spawn_once.py scripts/inspect_scene_once.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
