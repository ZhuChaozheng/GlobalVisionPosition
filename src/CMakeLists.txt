add_library(GlobalVisionLibrary SHARED 
	init_camera.cc
	image_transformation.cc
	image_process.cc
	car.cc
	udp.cc
	pid.cc
	timer.cc
	file_operation.cc
)
target_link_libraries(GlobalVisionLibrary ${THIRD_PARTY_LIBS})

add_executable(control_interface control_interface.cc)
target_link_libraries(control_interface GlobalVisionLibrary)

add_library(ControlInterfaceLibrary SHARED 
	control_interface.cc
)
target_link_libraries(ControlInterfaceLibrary GlobalVisionLibrary)

