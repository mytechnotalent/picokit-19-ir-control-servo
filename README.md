![picokit-19-ir-control-servo](https://raw.githubusercontent.com/mytechnotalent/picokit-19-ir-control-servo/main/picokit-19-ir-control-servo.png)

<br>

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/reverse-engineering)
## FREE Embedded Hacking Course [HERE](https://github.com/mytechnotalent/Embedded-Hacking)

<br>

# PICOKIT-19 IR CONTROL SERVO

### Remote Key Preset Angles on the SG90 Servo and Authenticated Heartbeat
#### Lesson 19 of the Picokit Series

<br>

***
**LEGAL DISCLAIMER:**
The information, tools, and code provided in this repository and course are strictly for educational, research, and defensive purposes only.

You are explicitly prohibited from using any materials contained herein to access, test, modify, or exploit any device, network, or system that you do not own 100% or for which you do not have explicit, documented, and legally binding authorization to interact with.

By using this repository and course, you acknowledge and agree that:

1. Any illegal, unauthorized, or malicious use of this information is solely your responsibility.
2. The author(s) and contributor(s) of this repository and course shall not be held liable for any damages, legal repercussions, criminal charges, or unauthorized actions resulting from the use, misuse, or abuse of the contents herein.
3. You will comply with all applicable local, state, national, and international laws regarding cybersecurity and computer fraud.

**IF YOU DO NOT AGREE WITH THESE TERMS, DO NOT USE THIS REPOSITORY AND COURSE.**
***

<br>
<br>

## Overview

The nineteenth Picokit lesson. The NEC infrared remote drives the SG90 servo
on GP14 to one of three preset angles, and every five seconds the node
transmits an authenticated heartbeat over LoRa to a Python gateway that logs
and displays the commanded angle. It shows how a decoded remote command can
command an actuator position.

<br>

## What it teaches

- Mapping distinct NEC command codes to zero, ninety, and one hundred eighty
  degrees.
- Driving the SG90 with a 50 Hz PWM pulse width derived from an angle.
- Ignoring a remote command that selects no preset.
- Sealing a tiny JSON body with Argon2id and XChaCha20-Poly1305 and sending it
  with an AT+SEND over the RYLR998.

<br>

## Hardware

| Peripheral | Pico 2 pin | Role |
| --- | --- | --- |
| VS1838B IR | GP5 | preset select input |
| SG90 servo | GP14 | commanded position |
| Red / Yellow / Green | GP16 / GP18 / GP17 | annunciator |
| Onboard LED | GP25 | heartbeat, one blink per transmit |
| RYLR998 | GP8 TX / GP9 RX | LoRa heartbeat |
| Debug Probe | SWCLK/SWDIO/GND, GP0/GP1 | SWD and the console |

<br>

## How it works

The node runs `monitor_step` in a loop. It re-applies the commanded position
about once a second, consumes console and infrared preset commands so that
`0x45` selects zero degrees, `0x46` selects ninety degrees, and `0x47` selects
one hundred eighty degrees, and every 5 seconds it seals
`{"n":19,"s":<seq>,"a":<angle>}` with the field key and sends it over LoRa.
The gateway authenticates each frame and only then parses it.

<br>

## Build and flash

```bash
cd firmware
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s
cmake --build build
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "program build/picokit_19_ir_control_servo.elf verify reset exit"
```

<br>

## Watch the node

Open the console at 115200 and reset:

```text
BOOT
=== PICOKIT-19 IR CONTROL SERVO // PRESET POSITION + HEARTBEAT ===
IR 0x46
POS angle=90 seq=0
IR 0x47
POS angle=180 seq=1
RX from 0x0001, N bytes
```

<br>

## The gateway

```bash
cd gateway
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 listen.py --port /dev/cu.usbserial-A50285BI --hub 0001 --network 18 --db gateway.db
```

It prints `OK node=19 rssi=...` per authenticated heartbeat. The terminal
dashboard `python3 tui.py --db gateway.db` and the web dashboard
`python3 web/app.py --db gateway.db` show the same rows.

<br>

## Verify

```bash
python3 .opencode/skill/embedded-c-standard/audit_c_standard.py
python3 .opencode/skill/embedded-python-standard/audit_python_standard.py
python3 .opencode/skill/iot-readme-standard/validate_readme.py
python3 .opencode/skill/iot-banner-standard/validate_banner.py
python3 scripts/run_tests.py
python3 scripts/check_coverage.py
```

<br>

# Next
[picokit-20-ir-lcd-status](https://github.com/mytechnotalent/picokit-20-ir-lcd-status)

<br>

# License
[MIT License](https://github.com/mytechnotalent/picokit-19-ir-control-servo/blob/main/LICENSE)
