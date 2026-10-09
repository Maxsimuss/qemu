# Original ESP32 APLL audio firmware

This independent ESP-IDF application runs the unmodified original-ESP32 I2S
driver and ROM analog-I2C functions. It configures the APLL through guest
MMIO, checks the calibration status, sends five seconds of stereo 16-bit PCM
through I2S0 DMA, and records resolved BCLK/WS/data pads with the virtual I2S
DAC. It has no host coefficient override or SDK interception.

With the selected ESP-IDF environment active, build either profile directly in
this directory:

```sh
./build.sh
APLL_DAC_RATE=32000 APLL_DAC_I2S_PORT=1 ./build.sh
```

The default profile uses I2S0 at 16 kHz. The second exercises I2S1 at 32 kHz.
Both send a complete one-kilohertz, five-second tone and check every captured
PCM sample, the stereo channels, DAC status, frame rate, and transmitted frame
count. Run the capture with a QEMU binary and flash image explicitly selected:

```sh
QEMU_BIN=/path/to/qemu-system-xtensa \
APLL_DAC_FLASH="$PWD/build/flash_image.bin" ./run.py

QEMU_BIN=/path/to/qemu-system-xtensa \
APLL_DAC_FLASH="$PWD/build/flash_image.bin" \
APLL_DAC_RATE=32000 APLL_DAC_I2S_PORT=1 ./run.py
```

The runner requires at least 0.2x realtime for the five-second tone and saves
the verified WAV and logs in this firmware project's ignored build directory.
From the workspace root, `./scripts/capture-dac.sh` is an optional convenience
that builds QEMU and the default firmware profile before capture. Build output,
the merged flash image, and WAV files are generated artifacts, not source
inputs.

The separate raw-MMIO qtests cover calibration cancellation, power loss,
reprogramming and re-enable behavior. A 10-microsecond calibration deadline
and the analog CAP/UDF/OVF results remain explicitly unverified silicon
approximations; the firmware validates the documented guest programming path
and downstream digital clock behavior.
