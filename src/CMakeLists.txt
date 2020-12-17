add_library(GlobalVisionLibrary SHARED 
	init_camera.cc
	2d_transformation_3d.cc
	image_process.cc
	car.cc
	udp.cc
	pid.cc
	timer.cc
)
target_link_libraries(GlobalVisionLibrary ${THIRD_PARTY_LIBS})
