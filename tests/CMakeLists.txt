include(FetchContent)

FetchContent_Declare(doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.12
  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(doctest)

add_executable(PathFinderTests
  BlockingsTest.cpp
  GridIndexTest.cpp
  LabelTest.cpp
  Main.cpp
  PathFinderTest.cpp
  RectTest.cpp
  TimeIntervalTest.cpp
  TraitsTest.cpp
)
target_link_libraries(PathFinderTests PRIVATE PathFinder::PathFinder doctest::doctest)

if ( MSVC )
  target_compile_options(PathFinderTests PRIVATE /W4 /permissive-)
else()
  target_compile_options(PathFinderTests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()
# GCC's -Wshadow also flags constructor parameters named like the members they initialize.
if ( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
  target_compile_options(PathFinderTests PRIVATE -Wshadow=local)
elseif ( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  target_compile_options(PathFinderTests PRIVATE -Wshadow)
endif()

add_test(NAME PathFinderTests COMMAND PathFinderTests)
