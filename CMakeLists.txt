cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Please define VITASDK to point to your SDK path!")
  endif()
endif()

project(VFPVector C)
include("${VITASDK}/share/vita.cmake" REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,-q -g -ftree-vectorize -Wall -O3 -nostdlib -fno-optimize-sibling-calls -e module_start")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")


add_executable(VFPVector
  src/vfp_vector.c
  src/vfp_vector.h
  src/vfp_f32_emu.c
  src/vfp_f64_emu.c
  src/vfp_gen.c
  src/main.c
)

target_link_libraries(VFPVector
  SceLibKernel_stub
  kubridge_stub
)

vita_create_self(VFPVector.suprx VFPVector CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/exports.yml)