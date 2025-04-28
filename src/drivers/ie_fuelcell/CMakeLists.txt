px4_add_module(
	MODULE drivers__ie_fuelcell
	MAIN ie_fuelcell
	COMPILE_FLAGS
	SRCS
		IE_Fuelcell.cpp
		IE_Fuelcell.hpp
	DEPENDS
		px4_work_queue
		systemlib
		uORB
	MODULE_CONFIG
		module.yaml
	)
