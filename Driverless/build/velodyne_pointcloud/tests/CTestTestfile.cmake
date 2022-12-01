# CMake generated Testfile for 
# Source directory: /home/mario/Driverless/src/velodyne/velodyne_pointcloud/tests
# Build directory: /home/mario/Driverless/build/velodyne_pointcloud/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_row_step "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/mario/Driverless/build/velodyne_pointcloud/test_results/velodyne_pointcloud/test_row_step.gtest.xml" "--package-name" "velodyne_pointcloud" "--output-file" "/home/mario/Driverless/build/velodyne_pointcloud/ament_cmake_gtest/test_row_step.txt" "--command" "/home/mario/Driverless/build/velodyne_pointcloud/tests/test_row_step" "--gtest_output=xml:/home/mario/Driverless/build/velodyne_pointcloud/test_results/velodyne_pointcloud/test_row_step.gtest.xml")
set_tests_properties(test_row_step PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/mario/Driverless/build/velodyne_pointcloud/tests/test_row_step" TIMEOUT "60" WORKING_DIRECTORY "/home/mario/Driverless/build/velodyne_pointcloud/tests" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/mario/Driverless/src/velodyne/velodyne_pointcloud/tests/CMakeLists.txt;4;ament_add_gtest;/home/mario/Driverless/src/velodyne/velodyne_pointcloud/tests/CMakeLists.txt;0;")
subdirs("../gtest")
