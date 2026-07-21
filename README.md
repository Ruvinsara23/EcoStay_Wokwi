# EcoStay complete Wokwi project + dynamic PZEM integration

This package replaces the temporary four-state test sketch with the uploaded
EcoStay firmware while retaining the dynamic Modbus PZEM model that already
passed the isolated test.

## Files

- `sketch.ino` — the EcoStay firmware with a
  Wokwi build target, explicit PZEM address `0x01`, and optional serial load
  commands.
- `diagram.json` — the complete ESP32, PZEM, relay, sensor, buzzer, switch,
  flow-input, water-input, and status-LED layout.
- `libraries.txt` — pinned Firebase, DHT, Adafruit Sensor, and PZEM libraries.
- `pzem-004t.chip.c` — dynamic PZEM model adapted to EcoStay's active-low
  relay signals.
- `pzem-004t.chip.json` — matching six-pin custom-chip definition.

The original uploaded `Recorrect_Code.ino` is not overwritten.

## Integration decisions

The uploaded firmware is the authority for the pin map:

| Function | ESP32 GPIO | Wokwi custom-chip pin |
|---|---:|---|
| PZEM data into ESP32 (RX2) | 18 | `TX` |
| PZEM data out of ESP32 (TX2) | 19 | `RX` |
| Exhaust-fan relay signal | 26 | `FAN` |
| Lighting relay signal | 13 | `LAMP` |

Remove the previous test connections from GPIO16/17 and GPIO25/26. In the
uploaded firmware, GPIO25 is the buzzer, not the fan signal.

The four-channel EcoStay relay logic is active-low (`LOW` = load ON). The
custom PZEM therefore uses active-low `FAN` and `LAMP` inputs and starts them
with pull-ups so that boot does not create a phantom load.

## Wokwi setup

1. Stop the simulation.
2. Replace the contents of your existing `wifi-scan.ino` with `sketch.ino`.
   Do not keep two `.ino` files containing `setup()` and `loop()` in the same
   project.
3. Replace `diagram.json` with the complete file in this package.
4. Add `libraries.txt`, `pzem-004t.chip.c`, and `pzem-004t.chip.json` as
   separate project files.
5. Build again. `libraries.txt` resolves the earlier
   `Firebase_ESP_Client.h: No such file or directory` error.

The complete diagram retains these six custom-chip connections:

   ```json
   [ "esp:5V",    "chip1:VCC",  "red",    [] ],
   [ "esp:GND.1", "chip1:GND",  "black",  [] ],
   [ "esp:18",    "chip1:TX",   "green",  [] ],
   [ "esp:19",    "chip1:RX",   "blue",   [] ],
   [ "esp:26",    "chip1:FAN",  "orange", [] ],
   [ "esp:13",    "chip1:LAMP", "purple", [] ]
   ```

6. Keep the exhaust relay `IN` connected to GPIO26 and the lighting relay `IN`
   connected to GPIO13. For the Wokwi relay contact animation, omit the
   `transistor` attribute or use `"transistor": "npn"` so a LOW input moves
   the contact to NO. The PZEM calculation itself does not require COM/NO/NC
   wiring.
7. Start the simulation. This sketch uses `Wokwi-GUEST` in simulation mode.

## Interactive simulation controls

| Part | Firmware input | How to test it |
|---|---|---|
| PIR | GPIO27 | Click it, then select **Simulate Motion** |
| Ultrasonic | GPIO17/16 | Click it and change the distance slider |
| DHT22 simulator | GPIO4 | Change temperature and humidity in its popup |
| MQ2 gas sensor | GPIO32 | Change its gas concentration |
| Water-level slider | GPIO34 | Move the slider; it substitutes for the analog water sensor |
| Flow clock | GPIO35 | Generates 16 pulses/s, representing about 3 L/min with the current calibration |
| Door switch | GPIO33 | Right = closed; left = open |
| Relays | GPIO26/14/13/5 | Controlled by Firebase or the PZEM load-test commands where applicable |

Wokwi supplies a DHT22 part, while the physical EcoStay prototype uses a
DHT11. `sketch.ino` selects DHT22 only when `ECOSTAY_WOKWI_SIMULATION` is `1`
and restores DHT11 when it is `0`.

## Isolated verification without Firebase

After boot, enter one command at a time in Serial Monitor. Allow up to three
seconds for the normal PZEM polling task to print/upload the new reading.

```text
SIM_LOAD OFF
SIM_LOAD LAMP
SIM_LOAD FAN
SIM_LOAD BOTH
```

Expected readings:

| Command | Voltage | Current | Power |
|---|---:|---:|---:|
| `SIM_LOAD OFF` | 235 V | 0.000 A | 0 W |
| `SIM_LOAD LAMP` | 235 V | about 0.051 A | 12 W |
| `SIM_LOAD FAN` | 235 V | about 0.298 A | 70 W |
| `SIM_LOAD BOTH` | 235 V | about 0.349 A | 82 W |

Energy increases at 60× time. It may appear to step when the next state is
shown because the model first accounts for energy consumed during the previous
state. The PZEM protocol reports whole watt-hours, so the firmware's kWh value
changes in approximately `0.001 kWh` increments.

Enter this to return fan and lamp control to Firebase:

```text
SIM_AUTO
```

## End-to-end Firebase verification

1. Provision the Wokwi ESP32 with the room-specific credentials returned by
   the EcoStay Admin page:

   ```text
   SET_CONFIG <propertyId> <roomId> <deviceEmail> <devicePassword>
   ```

2. Let it reboot and confirm `Using device credentials for Firebase Auth.`
3. With `SIM_AUTO` active, toggle **Lights** and **Exhaust Fan** from the
   dashboard.
4. Confirm that the relay GPIOs change and the PZEM reports 0/12/70/82 W.
5. For vacancy automation, enable `automationEnabled`, create a fresh
   transition into `VACANT_CONFIRMED`, and let the automation worker write
   `lights=false` and `exhaustFan=false`. The next PZEM read should be 0 W.

## Physical ESP32 build

Before flashing real hardware:

1. Set `ECOSTAY_WOKWI_SIMULATION` to `0`.
2. Replace `YOUR_WIFI_SSID` and `YOUR_WIFI_PASSWORD`, preferably through a
   local secrets header that is not committed.
3. Keep the real PZEM UART on ESP32 RX GPIO18 and TX GPIO19, matching this
   chosen pin map.
4. Do not treat `pzem-004t.chip.c` as physical-device firmware. It runs only in
   Wokwi.

## Evidence boundary

The dynamic custom chip is a deterministic software simulator. It demonstrates
that relay commands, Modbus reads, telemetry, accumulated-energy handling, and
the cloud data path can work together. It does **not** prove that a physical
PZEM measured real current, nor does it establish real-world energy savings.

The model currently assumes 235 V, unity power factor, a 70 W fan, and a 12 W
lamp. Replace the two wattage constants with measured/nameplate values before
using the simulation for a project estimate, and label results as simulated or
modelled.

## Behaviours intentionally left unchanged

This integration does not silently redesign unrelated application logic. In
the uploaded sketch, `mainRelay` is read but not applied, the presence relay is
driven by `motionDetection`, and the pump is a direct cloud command. Those
items should be handled as a separate control-policy change after this PZEM
integration passes.
