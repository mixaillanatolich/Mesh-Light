set(MODEL_COMMON_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/model_common.c" CACHE INTERNAL "")

set(MODEL_COMMON_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

set(lint_include_dirs
    ${MODEL_COMMON_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates
    ${CMAKE_SOURCE_DIR}/mesh/test/include)

add_pc_lint(model_common_${PLATFORM}
            "${MODEL_COMMON_SOURCE_FILES}"
            "${lint_include_dirs}"
            "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-D__WEAK=")
