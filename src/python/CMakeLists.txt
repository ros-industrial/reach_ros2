# Add Python libraries
python_add_library(${PROJECT_NAME}_python MODULE python_bindings.cpp)
target_include_directories(${PROJECT_NAME}_python PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_python
  PRIVATE ${PROJECT_NAME}
          Boost::python
          Boost::numpy
          ${PYTHON_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_python PRIVATE MODULE_NAME=_${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_python PROPERTIES OUTPUT_NAME _${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME}_python PUBLIC VERSION 17)

if(NOT PYTHON_INSTALL_DIR)
  message(FATAL ERROR "'PYTHON_INSTALL_DIR' must not be empty")
endif()
install(TARGETS ${PROJECT_NAME}_python DESTINATION ${PYTHON_INSTALL_DIR})
