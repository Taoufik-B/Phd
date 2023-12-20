# CMakeList.txt : CMake project for Sfml_PhysiX, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.

set(SFML_DIR "P:/02_Tools/LIBS/SFML-2.5.1/lib/cmake/SFML")

set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 2.5 COMPONENTS graphics window system)

add_executable (Sfml_PhysiX "Sfml_PhysiX.cpp" "Sfml_PhysiX.h")

# adding Target Link Libraries
target_link_libraries(Sfml_PhysiX 
	sfml-graphics
)


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET Sfml_PhysiX PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.



