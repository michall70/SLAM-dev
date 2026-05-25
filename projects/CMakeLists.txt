cmake_minimum_required(VERSION 3.10)
project(MyOrbbecApp)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/lib/orbbecsdk/include
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    /usr/local/lib
)

link_directories(
    ${CMAKE_SOURCE_DIR}/lib/orbbecsdk/lib
)

# add_executable(MyOrbbecApp src/extract.cpp)
# target_link_libraries(MyOrbbecApp OrbbecSDK)
# target_link_libraries(MyOrbbecApp ${OpenCV_LIBS})

# add_executable(DepthCalculator src/depth.cpp)
# target_link_libraries(DepthCalculator OrbbecSDK)
# target_link_libraries(DepthCalculator ${OpenCV_LIBS})

# add_executable(OrbExtractor src/ORB.cpp)
# target_link_libraries(OrbExtractor OrbbecSDK)
# target_link_libraries(OrbExtractor ${OpenCV_LIBS})

add_executable(TrajectoryCalculator src/trajectory.cpp)
target_link_libraries(TrajectoryCalculator OrbbecSDK)
target_link_libraries(TrajectoryCalculator ${OpenCV_LIBS})

add_executable(TrajectoryViewer src/plotTrajectory.cpp)
target_link_libraries(TrajectoryViewer
    pango_display
    pango_scene
)  
target_link_libraries(TrajectoryViewer ${OpenCV_LIBS})