# IE-SOAR S800 customer CAN format 2 (single system), extended ID 0x400.
# Receive-only telemetry. Not a DroneCAN data type.
uint64 timestamp                  # Publication time, microseconds since boot
uint64 timestamp_sample           # Last accepted frame reception, microseconds since boot
float32 tank_pressure_bar         # Cylinder gauge pressure in bar; NOT hydrogen percent
float32 battery_voltage_v         # Hybrid battery voltage in V
float32 output_power_w            # Combined output power in W
float32 stack_power_w             # SM input power as specified by the CAN protocol, W
float32 battery_power_w           # Signed hybrid battery power in W, raw * 10 - 5000
uint32 received_frames            # All accepted extended 0x400 frames with DLC 8
uint32 duplicate_frames           # Repeated counter observations, not necessarily duplicate payloads
uint8 interface                   # 1-based CAN interface (1 = CAN1 on Pixhawk 6C)
uint8 counter                     # Four-bit FCPM cyclic counter
uint8 state                       # Raw four-bit FCPM state
uint8 error_code                  # Raw six-bit CAN error code; no UART subcode is available
uint8[8] raw_data                  # Last accepted CAN payload, in wire byte order
bool connected                    # A matching frame arrived within timeout; not proof of valid decoding

# If connected is false, measurements are NaN. timestamp_sample retains the
# last valid reception time (zero before the first accepted frame).
