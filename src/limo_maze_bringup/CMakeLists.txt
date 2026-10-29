cmake_minimum_required(VERSION 3.8)
project(limo_maze_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES ../../worlds/example_maze.world ../../worlds/smoke_maze.world
        DESTINATION share/${PROJECT_NAME}/worlds)
install(PROGRAMS scripts/map_session.py DESTINATION lib/${PROJECT_NAME})
ament_package()
