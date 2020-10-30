add_library(GlobalVisionLibrary SHARED 
	InitCamera.cpp 
	ImageProcess.cpp
	2dTransformation3d.cpp
)
target_link_libraries(GlobalVisionLibrary ${THIRD_PARTY_LIBS})
