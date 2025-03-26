px4_add_module(
	MODULE drivers__ie_fuelcell
	MAIN ie_fuelcell
	COMPILE_FLAGS
		-DDEBUG_BUILD
	SRCS
		ie_fuelcell.cpp
		ie_fuelcell.hpp
		ie_fuelcell_main.cpp
	DEPENDS
		px4_work_queue
		systemlib
		uORB
	MODULE_CONFIG
		module.yaml
	)
