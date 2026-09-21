# RLogo Controller

This ESP32-S3 project connects to the `PowerRune` Wi-Fi AP and provides a
text command console over the board's built-in USB-JTAG virtual serial port.

The Controller connects to Master at `192.168.4.1:8080` and reconnects
automatically. Commands are newline-delimited and case-insensitive:

```text
HELP
STATUS
CONFIG color=red mode=big loop=off dir=cw
START
RUN color=blue mode=small loop=on dir=ccw
STOP
UNLOCK
OTA
REDLED ON
REDLED OFF
DEBUG ON
DEBUG OFF
THRESHOLD SET 1 35.00
THRESHOLD OFF 1
THRESHOLD SHOW
```

`RUN` with options sends the configuration and starts immediately. `START`
uses the last configuration. Normal output shows connection state, warnings,
errors, and key game events such as `HIT armour=... ring=...`. `DEBUG ON`
also prints detailed group and protocol information.

The direction accepts `cw`, `ccw`, or `cs`. `dir=cs` keeps the Motor stationary
while the Armour/game flow continues; automatic Motor unlock/start and the
standalone `UNLOCK` command are blocked in this mode.

`REDLED ON` enters Master ballistic debug mode and turns the Master light red
without starting the game, Armour, or Motor. `REDLED OFF` exits the mode and
turns the Master light off. `START`, `RUN`, and `UNLOCK` are rejected while the
mode is active.

`THRESHOLD SET` enables the Master-side second peak filter for one Armour and
saves it in NVS. The value is a normalized peak from `0.00` to `255.00` and
each Armour (1-5) has an independent setting. `THRESHOLD OFF` disables that
filter and persists the disabled state. Threshold changes are accepted only
when the game is stopped; use `THRESHOLD SHOW` to inspect the current settings.

Armour performs the first local hit detection and reports every local
candidate, including its peak. Master makes the authoritative decision during
the game window, sends the accepted hit to this serial console, and returns a
decision to the Armour for its hit light. If a decision ACK is not received
within 100 ms, Master prints a `HIT_ACK_TIMEOUT` warning here.
