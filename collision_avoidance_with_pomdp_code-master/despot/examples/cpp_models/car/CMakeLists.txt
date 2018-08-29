cmake_minimum_required(VERSION 3.5)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	car_model
)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS
	roscpp
	std_msgs
	message_runtime
	car_model
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	include
)

add_executable("${PROJECT_NAME}_car"
        src/Car.cpp
	src/main.cpp
		src/pomdp_model/CarState.cpp
		src/pomdp_model/CarBelief.cpp
		src/pomdp_model/CarPolicy.cpp
		src/pomdp_model/Observation.cpp
        include/Car.h
		include/pomdp_model/CarState.h
		include/pomdp_model/CarBelief.h
		include/pomdp_model/CarPolicy.h
		include/pomdp_model/Observation.h
		include/util/Printer.h
		src/util/Printer.cpp
		include/traffic_models/transition_model.h
		src/traffic_models/CarTransitionModel.cpp
		include/traffic_models/CarTransitionModel.h
		src/traffic_models/PedestrianTransitionModel.cpp
		include/traffic_models/PedestrianTransitionModel.h
		include/util/ros_utilities.h
		include/util/helper_functions.h src/util/FileWriter.hpp src/pomdp_model/CarParticleLowerBound.cpp include/pomdp_model/CarParticleLowerBound.h src/pomdp_model/CarScenarioUpperBound.cpp include/pomdp_model/CarScenarioUpperBound.h include/util/eigenmvn.h)

target_link_libraries("${PROJECT_NAME}_car"
	"${PROJECT_NAME}"
	${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#install(DIRECTORY "include/"
#	DESTINATION "${INCLUDE_INSTALL_PATH}/${PROJECT_NAME}"
#)

install(TARGETS "${PROJECT_NAME}_car"
	LIBRARY DESTINATION "${LIBRARY_INSTALL_PATH}"
	RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
