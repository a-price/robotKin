cmake_minimum_required(VERSION 2.6)
project(RobotKin)

set(CMAKE_BUILD_TYPE "Release")
#set( CMAKE_VERBOSE_MAKEFILE true )

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

option(HAVE_URDF_PARSE "add urdf parsing"  ON)

file(GLOB lib_source "src/*.cpp" "include/*.h")
list(SORT lib_source)

file(GLOB unit_tests_source "test/*.cpp")
LIST(SORT unit_tests_source)

if( HAVE_URDF_PARSE ) #---------------------------

    file(GLOB parse_source "parsing/*.cpp")
    set(lib_source ${lib_source} ${parse_source} )

    # Check if the user is set up to parse URDFs
    find_package( urdfdom QUIET )
    if( urdfdom_FOUND )
       MESSAGE(STATUS "Found urdfdom -- We will compile a URDF parser")
       find_package( urdfdom_headers REQUIRED )
       include_directories( ${urdfdom_headers_INCLUDE_DIRS} )
       include_directories( ${urdfdom_INCLUDE_DIRS} )

       #message(STATUS "${urdfdom_LIBRARIES}")

       link_libraries( ${urdfdom_LIBRARIES} )

       find_package(Boost REQUIRED COMPONENTS system filesystem thread)
       include_directories(${BOOST_INCLUDE_DIRS})
       link_libraries( ${BOOST_LIBRARIES} )

    else( urdfdom_FOUND )
       MESSAGE(STATUS "Could NOT find urdfdom -- We will NOT compile a URDF parser")
    endif( urdfdom_FOUND )

endif( HAVE_URDF_PARSE ) #---------------------------

# Define target

add_library(${PROJECT_NAME} SHARED ${lib_source})

if( HAVE_URDF_PARSE )
   if( urdfdom_FOUND )
        set_property( TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS "HAVE_URDF_PARSE" )
    endif( urdfdom_FOUND )
endif( HAVE_URDF_PARSE )

enable_testing()

message(STATUS "\n-- UNIT TEST: ")
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
foreach(utest_src_file ${unit_tests_source})
    get_filename_component(test_base ${utest_src_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${utest_src_file})
    target_link_libraries(${test_base} ${PROJECT_NAME})
    add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
    add_custom_target(${test_base}.run ${test_base} ${ARGN})
    add_dependencies(check ${test_base})
endforeach(utest_src_file)


# TODO: Why is this in here twice??

if( HAVE_URDF_PARSE )
    if( urdfdom_FOUND )
       set_property( GLOBAL
                     PROPERTY COMPILE_DEFINITIONS "HAVE_URDF_PARSE" )
    endif( urdfdom_FOUND )
endif( HAVE_URDF_PARSE )

install(FILES   include/Constraints.h
                include/Robot.h
                include/Frame.h
                include/Linkage.h
                include/urdf_parsing.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/RobotKin)



install(TARGETS ${PROJECT_NAME} DESTINATION ${LIBRARY_INSTALL_PATH})

