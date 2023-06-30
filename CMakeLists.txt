cmake_minimum_required(VERSION 2.8.3)
project(search_test)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(${PROJECT_NAME}_node src/search_test_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
