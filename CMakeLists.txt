cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
# Abilita sia C che CXX per permettere la generazione del target MPI::MPI_C
project(tracker C CXX)

# Imposta lo standard C++17 richiesto da PCL 1.15
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Mantiene -fPIC per le librerie condivise
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(MPI REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

file(GLOB tracker-SRC
    "src/tracker/KalmanFilter.cpp"
    "src/tracker/Tracklet.cpp"
    "src/tracker/Tracker.cpp"
    "src/viewer/Renderer.cpp"
    "src/CloudManager.cpp"
    "src/AreaCounter.cpp"
)

set(tracker-LIBS ${PCL_LIBRARIES} pthread)
add_library(tracker SHARED ${tracker-SRC})
target_link_libraries(tracker ${tracker-LIBS})

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(main demo/main.cpp)
target_link_libraries(main tracker)




