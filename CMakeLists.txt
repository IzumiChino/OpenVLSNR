# SPDX-License-Identifier: GPL-2.0-or-later

cmake_minimum_required(VERSION 3.16)

project(OpenVLSNR VERSION 3.0.0 LANGUAGES C)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(OPENVLSNR_BUILD_PLUTO_TOOLS "Build PlutoSDR TX and RX tools" ON)
set(IIO_ROOT "" CACHE PATH "Root directory of the libiio 0.x SDK")

set(OPENVLSNR_SOURCES
	src/bb_frame.c
	src/bch_decode.c
	src/bch_encode.c
	src/demodulate.c
	src/demodulator_top.c
	src/dvbs2x_init.c
	src/filter.c
	src/interleaver.c
	src/ldpc_decode.c
	src/ldpc_encode.c
	src/ldpc_tables.c
	src/modcod.c
	src/modulate.c
	src/modulator_top.c
	src/pilot_insert.c
	src/plheader_gen.c
	src/scrambler.c
	src/sync_afc.c
	src/sync_freq_coarse.c
	src/sync_freq_fine.c
	src/sync_phase.c
	src/sync_timing.c
	src/vlsnr_header_gen.c
	src/vlsnr_layout.c
	tables/ldpc_tables.c
	tables/ldpc_tables_extra.c
)

add_library(dvbs2x_vlsnr STATIC ${OPENVLSNR_SOURCES})
target_include_directories(dvbs2x_vlsnr PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
	target_compile_options(dvbs2x_vlsnr PRIVATE
		-Wall -Wextra -Werror
	)
endif()

if(NOT MSVC)
	target_link_libraries(dvbs2x_vlsnr PUBLIC m)
endif()

if(OPENVLSNR_BUILD_PLUTO_TOOLS)
	find_package(PkgConfig QUIET)
	if(PkgConfig_FOUND)
		pkg_check_modules(LIBIIO QUIET IMPORTED_TARGET libiio)
	endif()

	if(TARGET PkgConfig::LIBIIO)
		set(OPENVLSNR_IIO_TARGET PkgConfig::LIBIIO)
	else()
		find_path(IIO_INCLUDE_DIR iio.h
			HINTS "${IIO_ROOT}"
			PATH_SUFFIXES include
		)
		find_library(IIO_LIBRARY
			NAMES iio libiio
			HINTS "${IIO_ROOT}"
			PATH_SUFFIXES lib lib64
		)
		if(NOT IIO_INCLUDE_DIR OR NOT IIO_LIBRARY)
			message(FATAL_ERROR
				"libiio 0.x not found; set IIO_ROOT to its SDK directory")
		endif()
		add_library(openvlsnr_iio UNKNOWN IMPORTED)
		set_target_properties(openvlsnr_iio PROPERTIES
			IMPORTED_LOCATION "${IIO_LIBRARY}"
			INTERFACE_INCLUDE_DIRECTORIES "${IIO_INCLUDE_DIR}"
		)
		set(OPENVLSNR_IIO_TARGET openvlsnr_iio)
	endif()

	add_executable(openvlsnr-pluto-tx
		tools/openvlsnr-pluto-tx.c
		tools/pluto.c
	)
	add_executable(openvlsnr-pluto-rx
		tools/openvlsnr-pluto-rx.c
		tools/pluto.c
	)
	foreach(tool openvlsnr-pluto-tx openvlsnr-pluto-rx)
		target_include_directories(${tool} PRIVATE tools)
		target_link_libraries(${tool} PRIVATE
			dvbs2x_vlsnr ${OPENVLSNR_IIO_TARGET}
		)
		if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
			target_compile_options(${tool} PRIVATE
				-Wall -Wextra -Werror
			)
		endif()
	endforeach()
endif()
