# ESP32 APLL implementation and acceptance plan

Scope: original ESP32, guest-visible hardware programmed by unmodified ROM and
ESP-IDF. Existing host-configured coefficients are a test fixture, not acceptance
evidence. No firmware hooks, driver-call interception, or unconditional CAL_END.

## Evidence before behavior

Use the ESP32 TRM, Espressif SDK register definitions and clock implementation,
and disassembly of the supplied original ESP32 mask ROM. Record exact source
locations and supported silicon revisions alongside code or tests. Distinguish
documented digital behavior from inferred analog timing. An unknown must not be
turned into an invented silicon guarantee.

ROM evidence already identified by independent review: rom_chip_i2c_readReg at
0x40004110 and rom_chip_i2c_writeReg at 0x40004168 access 0x6000e000 + 4*host.
The command contains block[7:0], register[15:8], data[23:16], write[24], busy[25].
APLL uses host 3. ANA_CONFIG_REG is 0x6000e044; reset field [17:8], with bit 14
cleared to enable the APLL bus. Recheck these against ROM before implementation.
SDK calibration writes register 0 with 0x0f, 0x3f, 0x1f, then polls register 3
bit 7. Resolve the meaning and evidence for each transition before modeling it.

The independent ROM audit compared the official revision-300 ELF code bytes
with the bundled QEMU ROM. Map both 0x3ff4e000 and 0x6000e000 bus aliases.
APLL block is 0x6d on host 3. Registers: 7 SDM2[5:0], 8 SDM1, 9 SDM0,
4 ODIV[4:0], 5 SDM_STOP[5]/RSTB[6], 0 CAL_DELAY[3:0]/RSTB[4]/START[5]/
UNSTOP[6]/ENB_FCAL[7], 3 CAL_CAP[4:0]/UDF[5]/OVF[6]/END[7]. Revision is
eFuse BLK0_RDATA3 bit 15, using the same source as the SDK. Audit RTC
BIAS_I2C_FORCE_PD (OPTIONS0 bit 18) and force/sleep behavior as well as ANA_CONF.

Primary evidence: Espressif's esp-rom-elfs repository; ESP-IDF master
components/soc/esp32/include/soc/regi2c_defs.h and regi2c_apll.h;
components/esp_hal_clock/esp32/include/hal/clk_tree_ll.h; TRM 7.2.4/7.2.7.
The present audit has no primary evidence for analog register reset values. QEMU
currently resets the ten-byte APLL analog bank to zero as an implementation
placeholder, not a silicon claim. Evidence is also absent for bus transaction
duration, calibration cycle count, varactor transfer law,
CAL_CAP/UDF/OVF results, or physical lock phase. These are outstanding evidence
requirements. A deterministic nominal model may enable SDK execution, but must
not be described as complete silicon fidelity without resolving those gaps.

## Implementation sequence

1. Replace the analog-I2C unimplemented region with a real MMIO bridge. Model
   commands, completion/busy, bus reset and addressing. Implement APLL analog
   registers, masks and readback. Unsupported analog blocks must remain explicit;
   never impersonate a working BBPLL or sensor with fabricated success.
2. Implement the calibration and power state machine. Register writes, reset,
   start, completion, power loss and reconfiguration must affect state. Establish
   completion timing from evidence; document any bounded approximation rather
   than claiming analog lock accuracy. Test unsuccessful and interrupted flows.
3. Connect programmed coefficients to the existing SoC clock tree. Derive crystal
   and silicon revision from shared chip configuration, not inconsistent device
   properties. Preserve revision-0 fractional-field semantics and force-PD
   precedence. Use exact rational nominal frequency through dividers where
   possible; accumulated pad phase must not drift from integer-MHz truncation.
4. Audit every consumer: CPU and APB source/divider selection, FRC/TIMG/watchdog,
   both I2S controllers, REF_TICK/CLK_OUT and physical pin routing. Cover power,
   reset, sleep/wakeup and source changes. Repair clock consumers within this
   scope; identify unsupported whole-SoC behavior explicitly.
5. Integrate reset and save/load state coherently. Reconstruct derived clocks and
   deadlines after load; never serialize host pointers or stale routing caches.
   Add useful guest-error diagnostics, upstream-style tests and documentation.

## Acceptance gates

- Raw MMIO qtests exercise the ROM command layout, both access aliases, readback,
  masks, busy/completion, ANA_CONFIG reset, calibration transitions, invalid
  configurations, revision behavior, and power/reset/reprogramming sequences.
- Tests independently compute expected rational clock periods and timer counts.
  Both I2S pad traces must match those periods, including clock loss/recovery.
- Compile independent firmware through workspace ccache. Run the stock ESP-IDF
  APLL programming path and mask-ROM analog-I2C functions without host coefficient
  overrides. Firmware must progress through calibration and generate I2S audio.
- Decode audio with the external DAC from resolved physical pad edges. Verify
  frame count, waveform, sample rate and transition timing; retain the WAV and
  capture command under /workspace. Include a second sample-rate configuration
  and a disable/re-enable or live reconfiguration case.
- Run existing ESP32 GPIO, interrupt-matrix, I2C, I2S, DAC, APLL and related timer,
  idle batching and migration regressions. Preserve at least 0.2x realtime for
  the five-second audio workload, including complete physical output edges.
- Independent reviewer checks diff, source evidence and observed firmware output.
  Fix findings before committing and pushing. Production readiness requires every
  required observable behavior above to pass; list any unverified silicon timing
  as a remaining limitation, not a passed gate.

## Ownership

Root writes this plan and handles commits; the user pushes from the host. The
initial implementation used Luna-high. Future subagents use Luna-medium, with at
most two active. Keep progress concise; no separate prep report.

## Current acceptance status (2026-10-09)

The documented digital paths now have raw-MMIO and guest-level coverage. Final
focused regressions passed: APLL 27, raw MMIO 12, REF_TICK/UART 10; GPIO 4,
interrupt controller 3, I2C 16, I2S 57 and DAC 17; RTC/GPIO/I2S/I2C isolated
migration 3/4/3/3; timer 1 and idle batching 6. The independent dual-core TCG
clock-gate check passed with WFI, reset and pending-interrupt behavior. ROM
analog-I2C access, calibration sequencing/cancellation, force-power handling,
both I2S rational clocks, CLK_OUT pad edges, CPU/APB source loss and recovery,
REF_TICK register/rate selection, UART REF_TICK pacing and timeout rescaling,
shared RTC calibration clocks, and isolated RTC/I2S/GPIO timer VMState are
implemented and covered by their focused tests. The independent full-tone audio
captures passed on I2S0 at 16 kHz (80,000 tone frames; 0.753x realtime) and I2S1
at 32 kHz (160,000 tone frames; 0.232x realtime). Both include firmware-driven
I2S disable/re-enable, and the verifier checked the complete waveform, stereo
samples, and zero padding. The IDF command audit found 0 of 1,045 application and bootloader compile
commands bypassing ccache. The rate-specific WAVs are retained in the fixture's
ignored build directory; `tests/firmware/esp32-apll-dac/README.md` gives the
reproduction commands.

The 10 microsecond calibration completion delay is still an unverified virtual
time approximation. CAL_CAP, UDF and OVF remain placeholders; analog lock,
capacitor calibration and physical phase are not established. Deep-sleep entry
and wakeup are not implemented by the ESP32 machine, so sleep-specific APLL
power transitions have not passed acceptance. RMT and LEDC are not modeled and
have no REF_TICK consumers. Xtensa whole-machine live migration is unsupported;
only isolated RTC, I2S and GPIO timer VMState streams have been validated.

Current release scope requires hardware-equivalent behavior observable by the
firmware: registers, timing, clocks and physical pins. The user expects deep
sleep to be unused; sleep-specific acceptance is outside this scope while that
remains true. Whole-machine snapshot/live migration is not a release requirement.
Its absence does not block OTA or ordinary firmware execution tests.

The remaining APLL fidelity questions are the unverified calibration timing,
status results and analog register reset values. They require hardware-equivalent
behavior where observable by firmware. Passing the stock SDK programming and
audio tests establishes those tested paths, not every possible observation of
the calibration hardware.
