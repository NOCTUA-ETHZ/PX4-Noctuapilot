/**
 * Fuel cell interface
 *
 * UART requires FC_SER_CFG and customer serial format 4, 5 or 6.
 * CAN requires UAVCAN_ENABLE > 0, matching UAVCAN_BITRATE and customer
 * CAN format 2. Reboot after changing interface or serial settings.
 *
 * @value 0 Disabled
 * @value 1 UART
 * @value 2 CAN1
 * @value 3 CAN2
 * @reboot_required true
 * @group Fuel Cell
 */
PARAM_DEFINE_INT32(FC_INTERFACE, 0);

/**
 * Fuel cell telemetry timeout
 *
 * Missing telemetry clears connected and invalidates measurements.
 * No flight action is taken. Must exceed the telemetry update interval.
 *
 * @unit ms
 * @min 200
 * @max 5000
 * @reboot_required true
 * @group Fuel Cell
 */
PARAM_DEFINE_INT32(FC_TIMEOUT, 2000);

/**
 * Fuel cell UART baud rate
 *
 * @value 9600 9600
 * @value 19200 19200
 * @value 38400 38400
 * @value 57600 57600
 * @value 115200 115200
 * @reboot_required true
 * @group Fuel Cell
 */
PARAM_DEFINE_INT32(FC_BAUD, 9600);
