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

The silicon documentation and available SDK do not establish the APLL
calibration duration, output capacitor calibration result, or underflow and
overflow behavior. The current digital model schedules CAL_END 10 microseconds
after a valid SDK START pulse. This bounded virtual-time approximation is
explicitly unverified; CAL_CAP, UDF and OVF remain zero placeholders. A passing
firmware clock/audio test therefore establishes the guest programming path and
downstream digital clock routing, not analog lock accuracy or production-level
silicon calibration fidelity.
