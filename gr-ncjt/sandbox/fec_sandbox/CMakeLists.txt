cmake_minimum_required(VERSION 3.10)
project(PolarEncoderExample CXX)

# Find pkg-config to locate GNU Radio FEC.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GNURADIO_FEC REQUIRED gnuradio-fec)

# Find Boost (header-only usage is sufficient in this case)
find_package(Boost REQUIRED)

# Include GNU Radio FEC headers and Boost headers.
include_directories(${GNURADIO_FEC_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Optionally add any additional compiler flags.
add_definitions(${GNURADIO_FEC_CFLAGS_OTHER})

# Create the executable from the main source file.
add_executable(encoder_example main.cpp)

# Link the GNU Radio FEC libraries.
target_link_libraries(encoder_example ${GNURADIO_FEC_LIBRARIES})


