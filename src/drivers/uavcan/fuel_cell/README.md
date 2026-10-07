# Fuel cell CAN backend

Internal raw-frame receiver for the unified `fuel_cell` driver.
See `src/drivers/fuel_cell/README.md` for configuration, protocol limitations,
message compatibility and tests.

This backend owns libuavcan's raw receive listener while FC_INTERFACE is
CAN1 or CAN2. It publishes only `fuel_cell_can`; the common driver publishes
`fuel_cell`. Listener installation fails if another component owns the slot.
All listener/update operations run under the owning UAVCAN node mutex.
