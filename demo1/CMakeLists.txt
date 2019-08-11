# create an executable
ADD_EXECUTABLE (demo1 main.cpp ${COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (demo1 ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/demo1)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf ../resources/5DOF_Scara/robot.urdf  DESTINATION ${APP_RESOURCE_DIR})