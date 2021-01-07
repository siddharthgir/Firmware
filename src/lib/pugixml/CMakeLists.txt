cmake_minimum_required(VERSION 3.4)
project(pugixml VERSION 1.11 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(CMakeDependentOption)
include(GNUInstallDirs)

px4_add_library(pugixml
	src/pugixml.cpp
)


