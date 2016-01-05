CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
INCLUDE(cmake/base.cmake)
SET(PROJECT_NAME hpp_dynamic)
SET(PROJECT_DESCRIPTION
  "test for dynamic path planning"
)
SET(PROJECT_URL "")
SETUP_PROJECT()
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/dyn/dynPlanner.hh
)
# Create and install executable running the corba server
ADD_EXECUTABLE (hpp-dyn-server
  src/dynPlanner.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-dyn-server hpp-corbaserver)
# Install executable
INSTALL (TARGETS hpp-dyn-server DESTINATION ${CMAKE_INSTALL_BINDIR})
SETUP_PROJECT_FINALIZE()
