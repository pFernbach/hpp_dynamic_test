CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)
SET(CXX_DISABLE_WERROR TRUE)
SET(PROJECT_NAME hpp_dynamic)
SET(PROJECT_DESCRIPTION
  "test for dynamic path planning"
)
SET(PROJECT_URL "")
SETUP_PROJECT()

FIND_PACKAGE(catkin)
CATKIN_PACKAGE()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()


ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")


# This dependency ensures us that the intermediate __init__.py files are
# already installed.
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 2.1")
# Make sure that python submodule is installed in the same prefix as module.
IF (NOT ${HPP_CORBASERVER_PREFIX} STREQUAL ${CMAKE_INSTALL_PREFIX})
  MESSAGE(WARNING
    "  THIS PACKAGE SHOULD BE INSTALLED IN THE SAME PREFIX AS HPP-CORBASERVER,\n"
    "  SINCE IT INSTALLS A PYTHON SUBMODULE OF HPP.CORBASERVER.")
  MESSAGE("CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
  MESSAGE("HPP_CORBASERVER_PREFIX=${HPP_CORBASERVER_PREFIX}")
ENDIF()
SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  urdf/robot_box.urdf
  urdf/box.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  srdf/robot_box.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )

FINDPYTHON()

install (FILES
  src/hpp/corbaserver/box/robot_box.py
  src/hpp/corbaserver/box/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/box)

SET(${PROJECT_NAME}_HEADERS
  include/hpp/dyn/dynPlanner.hh
  include/hpp/dyn/steering-dynamic.hh
)
# Create and install executable running the corba server
ADD_EXECUTABLE (hpp-dyn-server
  src/dynPlanner.cc
  src/hpp-dyn-server.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-dyn-server hpp-corbaserver)
# Install executable
INSTALL (TARGETS hpp-dyn-server DESTINATION ${CMAKE_INSTALL_BINDIR})

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})


SETUP_PROJECT_FINALIZE()
