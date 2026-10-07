ESP32 digital peripheral testing
===============================

Run original ESP32 firmware with the ``esp32`` machine. The I2C and I2S
controllers access guest registers, FIFOs and DMA memory. GPIO matrix/IOMUX
routing resolves their digital signals together with pulls and external drives.

Capture pads without a graphical interface::

  qemu-system-xtensa -machine esp32 -display none -nic none \
    -icount shift=0,align=off,sleep=off \
    -drive file=flash.bin,if=mtd,format=raw \
    -global driver=esp32.gpio,property=pin-trace,value=pins.vcd \
    -d unimp,guest_errors -D device.log

``pins.vcd`` records resolved GPIO nodes, the ESP32's own drive, and combined
external drive at 1 ns virtual resolution. ``0/1/z/x`` mean low, high, release,
and unknown/contention. GPIO and QFN48-plus-exposed-pad terminal labels alias the
same digital nodes. Non-digital terminals explicitly retain ``UNMODELED``/``x``;
this inventory does not provide analog/RF/power-domain fidelity. Trace write
failure terminates the run with a nonzero host status. Keep a pristine flash
image and run tests against a copy when comparing repeatable boot behavior.

The named GPIO input ``pad-drive`` supplies four independent external strong
or released drives: input index = driver * 40 + GPIO number, level = 0/1/2/3
for low/high/release/unknown. ``pad-level`` reports the resolved node. Peripheral
inputs receive GPIO matrix/IOMUX results; externally driving a node does not
replace the ESP32's drive. I2C fixtures attach explicitly with
``-device esp32-i2c-peer,gpio=/machine/soc/gpio,sda=21,scl=22,address=80``.

Run device tests from the configured QEMU build::

  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-i2s-test
  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-gpio-test
  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-intmatrix-test
  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-i2c-test
  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-i2s-dac-test
  QTEST_QEMU_BINARY=./qemu-system-xtensa tests/qtest/esp32-apll-test
  tests/unit/test-qemu-timer
  tests/unit/test-icount-idle
  tests/unit/test-esp32-i2s-vmstate
  tests/unit/test-esp32-i2c-vmstate

For firmware audio capture, use an optimized build and fixed
``-icount shift=7,align=off,sleep=off``. This gives 128 ns instruction quanta;
use ``shift=0`` separately for nanosecond CPU/timer checks. In precise no-sleep
mode, the main-loop processes successive idle virtual deadlines without a CPU
thread round trip for each edge. Every timer callback and virtual deadline is
retained. A batch stops for CPU interrupt/work, pending main-loop work,
256 deadlines, or 1 ms of host time. Replay, qtest and sleeping modes retain
their normal dispatch. This optimization changes host execution cost rather
than peripheral clock divisors or sample data.

These models remain incomplete. Do not treat passing tests as certification
of all legal firmware:

* Exact PDM filter arithmetic, CVSD/PLC, internal ADC/DAC, FIFO power-down
  retention, nonzero TIMING delay/double-sync fields, DMA hung/cmdFIFO/loop-test
  operation and DMA AHB/burst/arbitration timing are unimplemented or unverified.
  Unknown active I2S outputs produce ``x`` and unsupported transfers produce no
  fabricated DMA completion. Exercised absent modes emit default stderr
  capability diagnostics. Enable ``unimp`` logging for detailed register access.
* TRM v5.8 and SDK disagree about LCD_EN reset and FIFO reset-back status.
  The register table retains SDK values; reset-back transitions are unverified.
* APLL currently supplies a host-configured frequency fixture, not a
  guest-programmable APLL. RTC_CNTL_ANA_CONF_REG implements its documented
  mask/reset and force-power-down precedence. The nominal formula uses
  SDM0/SDM1 on revision 1 and later; revision 0 ignores them (CLK-3.7).
  The 350-500 MHz numerator and ODIV range permit outputs below the TRM
  overview's 16 MHz lower bound; official ESP-IDF supports approximately
  5.303031-125 MHz. The model retains the TRM's strict numerator boundaries;
  SDK coefficient search uses inclusive boundaries, so exact endpoints remain
  a specification discrepancy. Fractional frequency is truncated to integer Hz.
  CPU/APB derivation for SOC_CLK_SEL=3 and I2S_CLKA_ENA N+b/a and M dividers
  use this fixture. Coefficients and crystal/revision choices are independent
  machine properties (apll-sdm0/1/2, apll-odiv, apll-xtal-hz, apll-rev0),
  rather than guest registers, actual board crystal or eFuse revision inputs.
  The analog MMIO region remains an unimplemented device. Official SDK APLL
  configuration writes use analog-I2C/ROM and poll CAL_END: that programming
  and calibration path does not work here. No successful CAL_END is invented.
  Sleep-follow gating, SYSCON REF_TICK dividers, APLL CLK_OUT pin toggling,
  and RTC/APLL migration state remain explicit gaps.
  CPU/APB behavior with a selected APLL that subsequently loses power is also
  unverified; I2S source availability is invalidated independently of that mux.
  Run ``tests/qtest/esp32-apll-test`` for covered fixture/pad regressions.
* Frequency/divider tests verify cumulative rational periods. Divider duty-cycle
  details, clock-domain phase, live divider/configuration changes and unspecified
  cross-mode startup/stop sequences are not universally certified.
* Native routing is partial. Existing UART/SPI/other peripherals, RTC mux/sleep,
  straps, analog loading/drive strength and non-digital pad domains require
  further models. Unknown peripheral drives are not replaced with GPIO_OUT.
* Actual I2S VMState stream, timer restoration and peripheral signal re-drive
  have isolated tests. Xtensa CPU migration is explicitly unsupported upstream;
  remaining existing RTC/cache state also lacks serializers. Whole-chip
  snapshots or firmware continuation through migration are not supported.

Regenerate the checked-in register masks with the published SDK header::

  scripts/generate-esp32-i2s-registers.py /path/to/esp-idf/components/soc/esp32/register/soc/i2s_reg.h

The workspace's independent firmware runners are
``/workspace/esp32-sim/tests/i2s-firmware/run.py`` and
``/workspace/esp32-sim/scripts/test-i2c.sh``. Their source and decoded VCD checks
are separate from the device models; they certify their covered cases only.
