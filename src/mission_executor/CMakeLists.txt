cmake_minimum_required(VERSION 3.16)
project(mission_executor)

find_package(ament_cmake REQUIRED)

# Cargo workspace root is the repo root (see /Cargo.toml).
set(CARGO_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../target")
set(RUST_BINARY "${CARGO_TARGET_DIR}/release/${PROJECT_NAME}")

file(GLOB_RECURSE RUST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs"
)

add_custom_command(
  OUTPUT ${RUST_BINARY}
  COMMAND ${CMAKE_COMMAND} -E env
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    cargo build --release --quiet --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  DEPENDS ${RUST_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml ${CMAKE_CURRENT_SOURCE_DIR}/../../Cargo.toml ${CMAKE_CURRENT_SOURCE_DIR}/../../Cargo.lock
  COMMENT "Building Rust executable"
  VERBATIM
)

if(WIN32)
  set(SUFFIX ".exe")
else()
  set(SUFFIX "")
endif()
add_custom_target(build_rust ALL DEPENDS ${RUST_BINARY})

install(
  PROGRAMS ${RUST_BINARY}${SUFFIX}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
