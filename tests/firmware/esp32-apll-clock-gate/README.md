# Original ESP32 CPU clock-gate regression

The independent Xtensa assembly increments one guest RAM word per core. The
runner uses real analog-I2C and RTC/DPORT MMIO to program and select APLL,
power it down, and recover by selecting XTAL. Both counters must advance with
a valid source, freeze when the selected clock is absent, and resume afterward.
The fixture also enters WAITI on both cores, verifies clock recovery preserves
that halt state, verifies CPU software resets restart execution, and verifies
pending interrupt-matrix inputs wake both cores only after clock recovery.
QEMU's standard loader supplies CPU1's test entry point; DPORT MMIO controls
its clock and stall. No firmware APIs or model test properties are intercepted.
Host waits provide an observation window and do not specify hardware timing.

Build using an ESP32 compiler and run with an ESP32-enabled QEMU binary:

```sh
ESP32_CC=/path/to/ccache-esp32-gcc ./build.sh
QEMU_BINARY=/path/to/qemu-system-xtensa python3 check.py
```

`ESP32_CC` is a compiler executable path, permitting a ccache wrapper.
`ESP32_CLOCK_GATE_LOG` optionally selects the ordinary QEMU log file. Generated
ELF and default logs live in the ignored `build/` directory.
