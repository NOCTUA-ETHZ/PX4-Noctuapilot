# Existing fields retain their names and UART format 4-6 meanings.
uint64 timestamp
float32 tankpressure # Hydrogen remaining in percent (legacy name); NaN on CAN
float32 regpressure # Regulated inlet pressure in bar; NaN on CAN
float32 voltage # V
float32 outputpower # W; NaN if unavailable/unverified
float32 spmpower # W; NaN if unavailable/unverified
float32 battpower # W, negative = discharge; NaN if unavailable/unverified
int32 psustate # -1 if unavailable
int32 mainerror # UART main error; -1 if unavailable (including CAN)
int32 suberror # UART subcode; -1 if unavailable (including CAN)

uint64 timestamp_sample # Last accepted reception, microseconds since boot
float32 tank_pressure_bar # CAN tank gauge pressure; NaN on UART
int32 raw_can_error # Raw CAN error, not a UART mainerror; -1 on UART/stale
uint8 source # 0 disabled, 1 UART, 2 CAN1, 3 CAN2 (FC_INTERFACE)
bool connected # Fresh telemetry, not fuel-cell operating state
