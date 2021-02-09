#add_executable(globalVision main.cpp)
#target_link_libraries(globalVision GlobalVisionLibrary)

add_executable(demo demo.cpp)
target_link_libraries(demo GlobalVisionLibrary)

add_executable(demo_thresh demo_thresh.cpp)
target_link_libraries(demo_thresh GlobalVisionLibrary)

#add_executable(signal signal.cc)
#target_link_libraries(signal GlobalVisionLibrary)

add_executable(demo_debug demo_debug.cc)
target_link_libraries(demo_debug GlobalVisionLibrary)