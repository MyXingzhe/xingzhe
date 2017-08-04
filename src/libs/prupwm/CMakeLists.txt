#-----------------------------------------------------------------
# CMake file for the MRPT application:  2d-slam-demo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(pru-pwm-tester)

cmake_minimum_required(VERSION 3.5)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------
SET(SRCS
	prupwm_test.cpp 
 )

IF(WIN32)
	SET(SRCS ${SRCS} resource.rc)
ENDIF()

# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})


# Add the required libraries for linking:
INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
