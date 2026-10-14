############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Share Protobuf's imported targets with Gazebo.
find_package(Protobuf QUIET CONFIG)
if(NOT Protobuf_FOUND)
    find_package(Protobuf)
endif()

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    # Create library
    if(COMMAND protobuf_generate)
        add_library(px4_gz_msgs STATIC ${MSGS_PROTOS})
        protobuf_generate(TARGET px4_gz_msgs LANGUAGE cpp APPEND_PATH)
    else()
        # CMake before 3.13 provides only protobuf_generate_cpp (legacy ROS CI).
        protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})
        add_library(px4_gz_msgs STATIC ${PROTO_SRCS} ${PROTO_HDRS})
    endif()

    # Disable double-promotion warning for protobuf-generated code
    target_compile_options(px4_gz_msgs PRIVATE -Wno-double-promotion)

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    # Propagate Protobuf's required C++ standard and dependencies to Gazebo consumers.
    target_link_libraries(px4_gz_msgs PUBLIC protobuf::libprotobuf)
    if (Protobuf_VERSION VERSION_LESS "3.8")
        target_compile_options(px4_gz_msgs PRIVATE -Wno-error=float-equal)
    endif()

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
