# Flash and configure a Matek CAN-L431 through a Pixhawk

This procedure uses a Pixhawk 6C temporarily running ArduPilot as the CAN adapter.
No CANable is required. It assumes the Matek CAN bootloader is intact.
The user confirmed flashing and the parameter-file workaround worked on this
setup on 2026-11-16. This does not record completion of every RC-loss, loaded-CAN
or flight acceptance test.

## Choose firmware first

- RC input only: use stock **MatekL431-RC** AP_Periph firmware, not the generic
  MatekL431-Periph sensor image.
- RC input plus telemetry: use **MatekL431-RC-Noctua-Telemetry.bin** from the
  separately supplied **MatekL431-RC-Telemetry-Complete.zip**, and matching PX4
  telemetry firmware. Stock Matek firmware lacks `RC_TEL_NODE`.
- The custom Matek firmware/source package is maintained separately; it is not
  uploaded to this repository. Retain the package and its checksums for reuse.
- PX4 telemetry is on
  [feature/dronecan-crsf-telemetry](https://github.com/NOCTUA-ETHZ/PX4-Noctuapilot/tree/feature/dronecan-crsf-telemetry);
  the RC-input-only branch does not provide the return path.

Use the Matek **application .bin** in the DroneCAN updater. The Pixhawk **.px4**
file belongs in QGroundControl's flight-controller firmware updater.

## 1. Back up PX4 and temporarily install ArduPilot

1. Connect to QGroundControl and save the PX4 parameters from
   **Vehicle Setup > Parameters > Tools > Save to file**. Keep the matching
   custom PX4 firmware file too.
2. Disconnect motor/servo power. Disconnect Pixhawk USB and other power.
3. Open **Vehicle Setup > Firmware** in desktop QGroundControl, then reconnect
   USB when prompted. Select stable **ArduPilot / Plane** for **Pixhawk 6C**.
   Verify the board selection before installing.
4. Wait for completion and close QGroundControl so it releases the USB port.
   Aircraft calibration is unnecessary for this temporary CAN-adapter use.

If the Pixhawk already runs ArduPilot, skip the firmware switch.

## 2. Connect and enable CAN1

With power disconnected, connect Pixhawk CAN1 to the Matek:

| Pixhawk CAN1 | Matek CAN |
| --- | --- |
| 5 V | 5 V |
| CAN H | CAN H |
| CAN L | CAN L |
| GND | GND |

Use regulated 5 V (Matek specifies 4.5–5.5 V) and a 120-ohm termination at each
physical bus end, accounting for existing terminators. Do not assume USB powers
the peripheral rail: verify the Matek power LED and supply.

Connect Mission Planner normally to the Pixhawk's USB MAVLink COM port.
In **Config > Full Parameter List/Tree**, write:

| ArduPilot parameter (on Pixhawk) | Value |
| --- | --- |
| CAN_P1_DRIVER | 1 |
| CAN_D1_PROTOCOL | 1 |
| CAN_P1_BITRATE | 1000000 |

Reboot. If driver parameters are initially absent, reboot after setting
`CAN_P1_DRIVER`, reconnect, then configure them. Use the existing bus bitrate
if different, matching every device. These are temporary ArduPilot parameters,
not PX4 or Matek parameters.

## 3. Flash the Matek over CAN

1. Keep Mission Planner connected through the normal MAVLink connection.
2. Open **Setup > Optional Hardware > DroneCAN/UAVCAN**.
3. Click **MAVLink CAN1**. This route does not require configuring a separate
   USB SLCAN COM port or changing a SERIAL protocol.
4. Wait for the Matek node to appear. Select **Update firmware** in its row.
5. If asked to download firmware automatically, choose **No** and select the
   local Matek `.bin` selected above.
6. Keep power connected until upload finishes and the node returns to
   **OPERATIONAL**, with health **OK**.
7. Open that node's **Parameters**. For the custom telemetry build,
   `RC_TEL_NODE` should be present.

A node in **MAINTENANCE** may be running only its bootloader and need the
application uploaded before parameters are available. If no node appears,
check supply, CAN H/L, bus selection, bitrate and termination first.

## 4. Load and persist the Matek parameters

Download the appropriate linked file using GitHub's **Raw / Download raw file**
button, or create it in a text editor. In Notepad choose **All files** when
saving, so the extension is `.param`, not `.param.txt`.

- [matek_rc_input.param](matek_rc_input.param): stock RC firmware, input only.
- [matek_rc_telemetry.param](matek_rc_telemetry.param): custom telemetry firmware.

The telemetry file contains:

```text
CAN_NODE,42
RC_TEL_NODE,1
RC_PORT,2
RC_PROTOCOLS,512
RC1_PORT_OPTIONS,0
RC_MSGRATE,50
```

The input-only file omits `RC_TEL_NODE`. Both assume an unused Matek node ID
**42**. The telemetry source is Pixhawk CAN node **1**; this is a CAN node ID,
not its MAVLink system ID. Change these examples if those IDs are already used.
Keep the Matek's `CAN_BAUDRATE=1000000` to match the example bus.

In the **Matek's** parameter window:

1. Click **Load from file** and select the appropriate `.param`.
2. Confirm the changed values; accepted pending edits should appear green.
3. Click **Write Params** and confirm. Wait for the result.
4. Click **Commit Params**, then **Refresh Params** and check the values.
5. Power-cycle the Matek. Close its old parameter window and reopen the newly
   discovered **node 42**. Read back the values again.

`RC_PORT=2` selects UART3. `RC_PROTOCOLS=512` selects CRSF only; `1` is
automatic detection. `RC1_PORT_OPTIONS=0` is normal non-inverted UART.
`RC_MSGRATE=50` provides margin above PX4's 100 ms RC freshness timeout.

### Red edited cells / values reverting to zero

On the tested Mission Planner setup, manually editing `RC_TEL_NODE` made the
cell red and refreshing restored zero. Loading the same value from a parameter
file succeeded. Red is not the normal pending-change indicator: Mission
Planner's cell-change handler marks exceptions red, whereas accepted changes
are green. A cell-value conversion issue is suspected, not proven for every
Mission Planner version.

Close/reopen the parameter window, load the file, then write/commit and read
back as above. **Commit Params alone does not send unsent edits.**
The popup saying **Update Params — DON'T DO THIS IF YOU ARE IN THE AIR** is a
confirmation warning; its **Error** title alone does not prove a failed write.
If writing still fails, retain the exact failure message.

The active node ID can remain **125** while `CAN_NODE` displays **42** until
reboot. Reopen parameters at the new ID after power cycling; an old window
still targets the old ID.

## 5. Restore PX4 and connect the receiver

Close Mission Planner. Use QGroundControl **Firmware > Advanced settings >
Custom firmware file** to restore the matching Pixhawk 6C `.px4` firmware.
For the supplied telemetry package this is **Pixhawk6C-Noctua-Telemetry.px4**.
Restore your saved PX4 parameters, then check these overrides:

| PX4 parameter | Value |
| --- | --- |
| UAVCAN_NODE_ID | 1 |
| UAVCAN_SUB_RC | 1 |
| UAVCAN_RC_ID | 42 |
| UAVCAN_RC_TEL | 1, telemetry branch only |
| RC_CRSF_PRT_CFG | 0 |
| UAVCAN_BITRATE | 1000000 |

Keep `UAVCAN_ENABLE=2` for sensors/automatic allocation, or retain `3` when
using CAN actuators. Do not disable existing CAN ESC functionality.
Reboot both boards. `RC_CRSF_TEL_EN` concerns direct UART CRSF and does not
enable this CAN return path.

| ELRS receiver | Matek |
| --- | --- |
| TX | RX3 |
| RX | TX3 |
| GND | GND |
| Supply | Regulated 5 V |

Both UART wires are needed for telemetry. Enable ELRS telemetry on the radio
and select **Telemetry > Discover new sensors** in EdgeTX. Confirm RC movement,
switch mapping and telemetry. Follow the full [RC input bench checks](rc_input.md#bench-checks-before-use)
before flight; for telemetry, also follow the acceptance checklist in the
telemetry branch's `rc_telemetry.md`. A successful flash alone does not validate
failsafe behavior.

## References

- [Mission Planner DroneCAN updater](https://ardupilot.org/planner/docs/mission-planner-initial-setup.html#dronecan-uavcan-slcan)
- [QGroundControl firmware installation](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html)
- [Matek CAN-L431](https://www.mateksys.com/?portfolio=can-l431)
- [Stock MatekL431-RC firmware](https://firmware.ardupilot.org/AP_Periph/stable/MatekL431-RC/)
- [Mission Planner parameter editor source](https://github.com/ArduPilot/MissionPlanner/blob/master/Controls/DroneCANParams.cs)
