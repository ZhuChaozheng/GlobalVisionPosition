add_executable(globalVision main.cpp)
target_link_libraries(globalVision GlobalVisionLibrary)

#add_executable(test test.cpp)
#target_link_libraries(test GlobalVisionLibrary)

add_executable(demo demo.cpp)
target_link_libraries(demo GlobalVisionLibrary)

add_executable(demo_thresh demo_thresh.cpp)
target_link_libraries(demo_thresh GlobalVisionLibrary)