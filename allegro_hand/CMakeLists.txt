cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES handover_viz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS python-scipy python-matplotlib
)

catkin_install_python(PROGRAMS scripts/detect_pcan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Testing
## Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(src/allegro_hand)
  add_rostest(test/allegro_launch.test)
endif()
