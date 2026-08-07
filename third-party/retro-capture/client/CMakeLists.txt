# retro-capture/client — consumer client library.
#
# Deliberately standalone, with no dependency beyond libc and pthreads:
# consumers add_subdirectory() this directory directly and do not inherit
# retro-capture's librga / GBM / EGL / GLESv2 / libdrm requirements just to
# talk to the socket.
#
#   add_subdirectory(third-party/retro-capture/client retro-capture-client)
#   target_link_libraries(your-target PRIVATE retro-capture-client)
#
# The parent CMakeLists also builds it (plus its fake-daemon ctest and the
# test-consumer oracle on top of it), which keeps the library honest: it has
# to compile without any consumer's headers on the include path, and no
# change ships unbuilt.
cmake_minimum_required(VERSION 3.18)

if(NOT DEFINED PROJECT_NAME)
  project(retro-capture-client CXX)
endif()

add_library(retro-capture-client STATIC
  src/retro-capture-client.cpp
  src/retro-overlay-client.cpp
)

target_include_directories(retro-capture-client
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # retro-capture-protocol.h
)

target_compile_features(retro-capture-client PUBLIC cxx_std_20)
target_compile_options(retro-capture-client PRIVATE -Wall -Wextra -Wno-unused-parameter)
set_target_properties(retro-capture-client PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
target_link_libraries(retro-capture-client PUBLIC Threads::Threads)
