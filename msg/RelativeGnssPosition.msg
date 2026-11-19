# Relative position of the target docking point from the approaching vehicle's centre of mass.
# TARGET_RELATIVE type RADIO_BEACON in LOCAL_OFFSET_NED is decoded by mavlink_receiver.
uint64 timestamp # [us] PX4 receive time
uint64 timestamp_sample # [us] GNSS sample time in PX4 clock, or receive time if not synchronized
uint8 target_id # [-] Same target identifier used by vision
float32[3] rel_pos # [m] [@frame NED] target minus vehicle; north, east, down
float32[3] var # [m^2] [@frame NED] per-axis measurement variance
