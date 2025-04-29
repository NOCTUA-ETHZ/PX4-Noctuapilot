px4_add_module(
	MODULE drivers__sen0473
	MAIN sen0473
	COMPILE_FLAGS
	SRCS
		sen0473.cpp
		sen0473.h
	DEPENDS
		px4_work_queue
	)
