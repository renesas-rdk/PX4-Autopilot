cmake_minimum_required(VERSION 3.10)

if(ENABLE_LOCKSTEP_SCHEDULER)
	px4_add_library(lockstep_scheduler
		src/lockstep_scheduler.cpp
		src/lockstep_components.cpp
	)

	target_include_directories(lockstep_scheduler
		PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
	)

	px4_add_functional_gtest(SRC test/src/lockstep_scheduler_test.cpp LINKLIBS lockstep_scheduler)
endif()
