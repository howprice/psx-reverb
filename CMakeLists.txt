cmake_minimum_required(VERSION 3.22)

project(psx-reverb)

set(PSX_REVERB_TARGET "psx-reverb")

add_executable(${PSX_REVERB_TARGET})

set(SRC_LIST
	"src/ArrayHelpers.h"
	"src/ClassHelpers.h"
	"src/Helpers.cpp"
	"src/Helpers.h"
	"src/hp_assert.cpp"
	"src/hp_assert.h"
	"src/Log.cpp"
	"src/Log.h"
	"src/main.cpp"
	"src/MathsHelpers.h"
	"src/Parse.cpp"
	"src/Parse.h"
	"src/StringHelpers.cpp"
	"src/StringHelpers.h"
	"src/Types.h"
	".editorconfig"
)

target_sources(${PSX_REVERB_TARGET} PRIVATE ${SRC_LIST} )

# Require C++17 to support terse static_assert
target_compile_features(${PSX_REVERB_TARGET} PRIVATE cxx_std_17)

# Define DEBUG in debug builds
target_compile_definitions(${PSX_REVERB_TARGET} PRIVATE $<$<CONFIG:Debug>:DEBUG>)

# Define RELEASE in debug builds
target_compile_definitions(${PSX_REVERB_TARGET} PRIVATE $<$<CONFIG:Release>:RELEASE>)

set_property(TARGET ${PSX_REVERB_TARGET} PROPERTY COMPILE_WARNING_AS_ERROR ON)

# Compiler-specific options
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")

	target_compile_options(
		${PSX_REVERB_TARGET} 
		PRIVATE
		/W4 
		/wd4127 # conditional expression is constant
#		/we4062 # enumerator 'identifier' in switch of enum 'enumeration' is not handled
	)

	# Allow use of portable fopen (fopen_s is Windows only)
	target_compile_definitions(${PSX_REVERB_TARGET} PUBLIC _CRT_SECURE_NO_WARNINGS _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR)

elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # GCC

	target_compile_options(
		${PSX_REVERB_TARGET} 
		PRIVATE 
		-Wall 
		-Wextra 

		# This can be over-pedentic e.g. anonymous structs
		# -pedantic
		
		-Wno-unknown-pragmas # Can't do in source file. See https://stackoverflow.com/questions/12842306/suppress-wunknown-pragmas-warning-in-gcc 
		-Werror
		-Wno-gnu-zero-variadic-macro-arguments
	)

elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	
	target_compile_options(
		${PSX_REVERB_TARGET} 
		PRIVATE 
		-Wall 
		-Wextra 
		-pedantic
		-Wno-unknown-pragmas # Can't do in source file. See https://stackoverflow.com/questions/12842306/suppress-wunknown-pragmas-warning-in-gcc 
		-Werror
		-Wno-gnu-zero-variadic-macro-arguments
		
#		-Wno-unused-const-variable

		# Suppress error: ISO C++ prohibits anonymous structs [-Werror=pedantic] for R3000 CauseRegister union
#		-Wno-gnu-anonymous-struct -Wno-nested-anon-types
		)	
else()
	message(FATAL_ERROR, "Unknown compiler")
endif()

# Define grouping for source files in IDE project generation. This is just for Visual Studio IDE convenience.
# Without this, all .cpp files are in the "Source Files" filter, and all .h files are in the "Header files" filter.
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC_LIST}) # Requires CMake 3.8

# Keep all data for the repo in a single folder for convenience
set(DATA_DIR "${PROJECT_SOURCE_DIR}/data")

set_target_properties(
	${PSX_REVERB_TARGET} PROPERTIES
	VS_DEBUGGER_WORKING_DIRECTORY ${DATA_DIR}
)

# Default Visual Studio start up project
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PSX_REVERB_TARGET})
