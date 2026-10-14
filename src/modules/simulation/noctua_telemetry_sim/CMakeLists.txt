if(NOT PX4_BOARD STREQUAL "px4_sitl")
	message(FATAL_ERROR "noctua_telemetry_sim is only supported on PX4 SITL")
endif()
if(NOT NOCTUA_MAVLINK_SOURCE_DIR)
	message(FATAL_ERROR "The Noctua SITL demo requires NOCTUA_MAVLINK_SOURCE_DIR")
endif()

px4_add_module(
	MODULE modules__simulation__noctua_telemetry_sim
	MAIN noctua_telemetry_sim
	SRCS NoctuaTelemetrySim.cpp
	DEPENDS px4_work_queue
)
