find_package(Protobuf REQUIRED)

find_program(PROTOC NAMES protoc)
if (NOT EXISTS ${PROTOC})
  message(FATAL_ERROR "The protoc program was not found")
endif()
message(STATUS "Found protoc program: " ${PROTOC})

if (NOT DEFINED GEN_LANG)
  set(GEN_LANG "cpp")
endif()

if (NOT DEFINED PROTO_SEARCH_PATH)
  set(PROTO_SEARCH_PATH ${CMAKE_SOURCE_DIR})
endif()

# protoc flags
set(PROTO_CFLAGS)
list(APPEND PROTOC_FLAGS "--proto_path=${PROTO_SEARCH_PATH}")
list(APPEND PROTOC_FLAGS "--${GEN_LANG}_out=${CMAKE_BINARY_DIR}") 

list(APPEND proto_files 
  # main-module configs
  "gpir_dev/gp_planner/benchmark/TDR_OBCA/proto/planner_open_space_config.proto"
)

# Absolute paths of .proto files to be passed to protoc program
set(ABSProtoFiles)
# Absolute paths of generated .h and .cc 
set(ProtoHeaders)
set(ProtoSources)

include_directories(${ProtoHeaders})

foreach(FNAME ${proto_files})
  # relative directory for .proto file
  get_filename_component(PROTO_PATH ${FNAME} DIRECTORY)
  # .proto name
  get_filename_component(PROTO_NAME ${FNAME} NAME_WE)
  # absolute directory and name of generated .h and .cc file
  set(GENERATED_PROTO_PATH "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}")

  # message("
  #  proto_path: ${PROTO_PATH}\n 
  #  proto_name: ${PROTO_NAME}\n
  #  generated_proto_path: ${GENERATED_PROTO_PATH}\n
  #  ABSProtoFiles: ${PROTO_SEARCH_PATH}/${FNAME}\n 
  #  ")

  list(APPEND ABSProtoFiles "${PROTO_SEARCH_PATH}/${FNAME}")
  list(APPEND ProtoHeaders "${GENERATED_PROTO_PATH}.pb.h")
  list(APPEND ProtoSources "${GENERATED_PROTO_PATH}.pb.cc")
  install (FILES ${GENERATED_PROTO_PATH}.pb.h DESTINATION include/${PROTO_PATH})
endforeach()

# Generate protos
add_custom_command(
  COMMAND ${PROTOC} ${PROTOC_FLAGS} ${ABSProtoFiles}
  OUTPUT ${ProtoSources} ${ProtoHeaders}
  COMMENT "Generating proto messages for planning module ..."
)

add_library(benchmark_proto_tdr_obca ${ProtoHeaders} ${ProtoSources})
add_library(benchmark::proto_tdr_obca ALIAS benchmark_proto_tdr_obca)
target_include_directories(benchmark_proto_tdr_obca PUBLIC $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>)
target_link_libraries(benchmark_proto_tdr_obca protobuf)
