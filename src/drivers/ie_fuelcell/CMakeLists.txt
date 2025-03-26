px4_add_module(
	MODULE drivers__ie_fuelcell
	MAIN ie_fuelcell
	COMPILE_FLAGS
		-DDEBUG_BUILD
	SRCS
		IE_Fuelcell.cpp
	MODULE_CONFIG
		module.yaml
	)
