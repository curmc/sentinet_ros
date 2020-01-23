project(math)

file(GLOB MATH_SRC "${PROJECT_SOURCE_DIR}/*.cpp")

target_sources(sentinet
  PUBLIC
  ${MATH_SRC})
