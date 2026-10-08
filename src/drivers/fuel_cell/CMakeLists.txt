px4_add_module(
 MODULE drivers__fuel_cell
 MAIN fuel_cell
 SRCS FuelCell.cpp
 DEPENDS px4_work_queue
 MODULE_CONFIG module.yaml
)
