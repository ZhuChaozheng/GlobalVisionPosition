#add_executable(globalVision main.cpp)
#target_link_libraries(globalVision GlobalVisionLibrary)

add_executable(demo demo.cpp)
target_link_libraries(demo GlobalVisionLibrary)

add_library(GlobalVisionPositionLibrary SHARED 
	demo.cpp
)
target_link_libraries(GlobalVisionPositionLibrary ${THIRD_PARTY_LIBS})

#add_executable(demo_thresh demo_thresh.cpp)
#target_link_libraries(demo_thresh GlobalVisionLibrary)

#add_executable(signal signal.cc)
#target_link_libraries(signal GlobalVisionLibrary)

#add_executable(demo_debug demo_debug.cc)
#target_link_libraries(demo_debug GlobalVisionLibrary)

#add_executable(thread_demo thread_demo.cc)
#target_link_libraries(thread_demo GlobalVisionLibrary)

#add_executable(read_test read_test.cc)
#target_link_libraries(read_test GlobalVisionLibrary)

#add_executable(demo_navigation demo_navigation.cc)
#target_link_libraries(demo_navigation GlobalVisionLibrary)


add_executable(call_interface_demo call_interface_demo.cc)
target_link_libraries(call_interface_demo ControlInterfaceLibrary)
