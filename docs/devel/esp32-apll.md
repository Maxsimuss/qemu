# Original ESP32 APLL model

The model accepts the original ESP32 ROM analog-I2C command format at
`0x6000e000` and its `0x3ff4e000` alias. In the original mask ROM,
`rom_chip_i2c_readReg` at `0x40004110` and `rom_chip_i2c_writeReg` at
`0x40004168` address a host command word at `0x6000e000 + 4 * host`. The
command packs block in bits 7:0, register in 15:8, data in 23:16, write in
bit 24 and busy in bit 25. This was checked against the official Espressif
`esp32_rev300_rom.elf` at
`/workspace/esp32-sim/tools/espressif/tools/esp-rom-elfs/20260528/`.

ESP-IDF sources used for the APLL bank and digital fields are
`components/soc/esp32/include/soc/regi2c_defs.h`,
`components/soc/esp32/include/soc/regi2c_apll.h`,
`components/esp_hal_regi2c/esp32/include/hal/regi2c_ctrl_ll.h`, and
`components/esp_hal_clock/esp32/include/hal/clk_tree_ll.h`. The APLL bank is
block `0x6d`, host 3. `ANA_CONFIG_REG` at `0x6000e044` has reset bits 17:8;
clearing bit 14 enables APLL-bus commands. The SDK writes APLL frequency
registers, pulses register 0 through `0x0f`, `0x3f`, `0x1f`, then polls
register 3 bit 7. APLL fractional-field semantics come from the shared eFuse
`BLK0_RDATA3.CHIP_VER_REV1` bit, not a model-only APLL property.

Clock frequency derives from the programmed registers and shared crystal
frequency. The numerator and denominator remain rational through I2S divider
and pad-edge scheduling, so fractional hertz do not accumulate truncation
drift. Force-power-down takes precedence over force-power-up in RTC ANA_CONF.
Unsupported analog hosts/blocks log a guest error and do not share APLL state.

The ten-byte APLL analog register bank currently resets to zero in QEMU. This
is an explicit implementation placeholder: available primary evidence does not
establish the silicon reset defaults for these analog registers, and the model
does not claim those zero values are hardware reset values.

The digital CLK_OUT route follows TRM register 6.33: GPIO0/CLK_OUT1 uses
IO_MUX_PIN_CTRL[3:0]=6, GPIO3/CLK_OUT2 also requires IO_MUX_PIN_CTRL[7:4]=6,
and GPIO1/CLK_OUT3 also requires IO_MUX_PIN_CTRL[11:8]=6. The raw APLL rate is
driven at those resolved physical pads with rational edge scheduling. The
I2S MCLK routes retain their documented selector combinations (CLK1=0 for
I2S0 or 15 for I2S1; CLK2/3=0 to propagate I2S MCLK).

REF_TICK uses the four source-specific `SYSCON_*_TICK_CONF` registers at
0x3ff66004, 0x3ff66008, 0x3ff6600c, and 0x3ff6603c. The modeled rate is
APB divided by `TICK_NUM + 1`; reset values are 39, 79, 11, and 99. The
existing UART model consumes this rate when `UART_CONF0.TICK_REF_ALWAYS_ON`
selects REF_TICK, including baud-dependent RX pacing; its RX timeout follows
the TRM's APB-compensated formula. The original ESP32 RMT and LEDC peripherals
are not modeled here, so their documented REF_TICK clock selections have no
consumer in this QEMU machine.

The silicon documentation and available SDK do not establish the APLL
calibration duration, output capacitor calibration result, or underflow and
overflow behavior. The current digital model schedules CAL_END 10 microseconds
after a valid SDK START pulse. This bounded virtual-time approximation is
explicitly unverified; CAL_CAP, UDF and OVF remain zero placeholders. A passing
firmware clock/audio test therefore establishes the guest programming path and
downstream digital clock routing, not analog lock accuracy or production-level
silicon calibration fidelity.

Deep-sleep entry and wakeup are outside the current ESP32 machine model. With
both ANA_CONF force bits clear, APLL follows the active system state; because
this machine has no deep-sleep transition, it remains available while active.
The RTC `BIAS_I2C_FORCE_PD` bus gate and ANA_CONF force-PD/force-PU precedence
are modeled independently.
