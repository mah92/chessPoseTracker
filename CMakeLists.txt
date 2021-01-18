#Besm ALLAH
cmake_minimum_required(VERSION 2.8)
project( ChessPoseTracker)

#Libraries#################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./") #also search current directory for Find*.cmake modules
##OpenCV################
find_package( OpenCV REQUIRED )

if(OpenCV_FOUND)
	message("Found Opencv!")
else()
	message("Can't find Opencv!")
endif()

include_directories(
    "${OpenCV_INCLUDE_DIRS}"
    "${PROJECT_SOURCE_DIR}/"
)

set( mSOURCES 
	pugixml/pugixml.cpp main.cpp)

add_executable( ChessPoseTracker ${mSOURCES})

target_link_libraries( ChessPoseTracker ${OpenCV_LIBS} )


