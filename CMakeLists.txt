cmake_minimum_required(VERSION 3.5)
project(FMUs)

include(CheckCXXCompilerFlag)

function(enable_c_compiler_flag_if_supported flag)
    string(FIND "${CMAKE_C_FLAGS}" "${flag}" flag_already_set)
    if(flag_already_set EQUAL -1)
        check_cxx_compiler_flag("${flag}" flag_supported)
        if(flag_supported)
        endif()
    endif()
endfunction()

macro(FMU name)

    cmake_minimum_required(VERSION 3.5)

    project (${name})

    set(FMI_PLATFORM "unknown" CACHE STRING "Target platform for the built FMU")
    if(FMI_PLATFORM STREQUAL unknown)
        if(CMAKE_HOST_WIN32)
            if(CMAKE_CL_64)
                set(FMI_PLATFORM win64)
            else(CMAKE_CL_64)
                set(FMI_PLATFORM win32)
            endif(CMAKE_CL_64)
        elseif(CMAKE_HOST_APPLE)
            if(CMAKE_SIZEOF_VOID_P EQUAL 4)
                set(FMI_PLATFORM darwin32)
            else(CMAKE_SIZEOF_VOID_P EQUAL 4)
                set(FMI_PLATFORM darwin64)
            endif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        elseif(CMAKE_HOST_UNIX)
            if(CMAKE_SIZEOF_VOID_P EQUAL 4)
                set(FMI_PLATFORM linux32)
            else(CMAKE_SIZEOF_VOID_P EQUAL 4)
                set(FMI_PLATFORM linux64)
            endif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        else(CMAKE_HOST_WIN32)
            message(SEND_ERROR "FMI_PLATFORM is not set")
            set(FMI_PLATFORM not_supported)
        endif()
    endif()

    set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
    )
    foreach(CompilerFlag ${CompilerFlags})
        string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
    endforeach()

    include_directories(${FMI2SPECIFICATION} ../ ${SUNDIALS}/include)
    link_directories(${SUNDIALS}/lib)

    set(FMU_BINARY_PATH binaries/${FMI_PLATFORM})

    if (CMAKE_COMPILER_IS_GNUCC)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -Wno-unused-parameter")
    endif()

    # Compile model binary
    add_library(${name} MODULE ${name}.c)
    set_target_properties(${name} PROPERTIES PREFIX "")
    target_link_libraries(${name} sundials_cvode)
    target_link_libraries(${name} sundials_nvecserial)
    if(UNIX)
        target_link_libraries(${name} m)
    endif(UNIX)
    if(WIN32)
        #target_compile_definitions(${name} SUNDIALS_EXPORT=__declspec(dllexport))
        add_definitions(-DBUILD_SUNDIALS_LIBRARY)
    endif(WIN32)

    # Copy shared library
    add_custom_target(${name}_LIBRARY
        COMMAND ${CMAKE_COMMAND} -E copy
        "$<TARGET_FILE:${name}>"
        "${CMAKE_CURRENT_BINARY_DIR}/${FMU_BINARY_PATH}/$<TARGET_FILE_NAME:${name}>")

    add_dependencies(${name}_LIBRARY ${name})

    # Copy modelDescription.xml
    add_custom_target(${name}_DESCRIPTION ALL
        COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_SOURCE_DIR}/modelDescription.xml"
        "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml")

    # Create fmu archive
    add_custom_target(${name}_FMU ALL
        COMMAND ${CMAKE_COMMAND} -E tar "cfv" "${name}.fmu" --format=zip
        "${FMU_BINARY_PATH}/$<TARGET_FILE_NAME:${name}>" "modelDescription.xml")

    add_dependencies(${name}_FMU ${name}_DESCRIPTION ${name}_LIBRARY)

    # Installation
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}.fmu" DESTINATION "${CMAKE_INSTALL_PREFIX}")

endmacro()

#[[
add_subdirectory(PT1)
add_subdirectory(PT2)
add_subdirectory(PI)
add_subdirectory(OscillatorD2D)
add_subdirectory(OscillatorF2D)
add_subdirectory(OscillatorD2F)
add_subdirectory(OscillatorOmega2Tau)
add_subdirectory(OscillatorTau2Omega)
add_subdirectory(Step)
add_subdirectory(Gain)
add_subdirectory(Subtraction)
add_subdirectory(Zero)
add_subdirectory(TwoMassOscillator)
add_subdirectory(ControlLoopPIxPT1)
]]
add_subdirectory(TwoMassRotationalOscillator)

macro(CopyFile name)
    # Copy file to build dir
    add_custom_target(${name} ALL
        COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_SOURCE_DIR}/${name}"
        "${CMAKE_CURRENT_BINARY_DIR}/${name}")

    # Copy file to install dir
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}" DESTINATION "${CMAKE_INSTALL_PREFIX}")
endmacro()

CopyFile(configuration.xsd)
CopyFile(ControlI.xml)
CopyFile(ControlIReference.xml)
CopyFile(Control10x.xml)
CopyFile(Control10xReference.xml)
CopyFile(TwoMassOscillatorD2D.xml)
CopyFile(TwoMassOscillatorF2D.xml)
CopyFile(TwoMassOscillatorReference.xml)
CopyFile(StepSubtraction.xml)
CopyFile(StepSubtractionReference.xml)

