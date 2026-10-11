ESP32 simulator operator quick start
====================================

With QEMU build dependencies and ccache installed, run from the checkout::

  mkdir -p build
  cd build
  ../configure --target-list=xtensa-softmmu --enable-gcrypt --enable-slirp \
    --enable-download --disable-docs --extra-cflags=-O3 \
    --cc="ccache cc" --cxx="ccache c++"
  ninja qemu-system-xtensa
  cd ..

Start from a complete 4 MiB raw SPI flash image containing the bootloader,
partition table and apps; an app-only ``.bin`` is not bootable. The helper
copies it into a new writable run directory and refuses nonempty directories::

  scripts/esp32-sim start /path/to/complete-flash.bin "$PWD/run"
  curl http://127.0.0.1:18080/
  curl http://127.0.0.1:18081/

The Wi-Fi peer defaults to ``test-ap`` / ``test-password``; override with
``--peer-ssid`` / ``--peer-password``. Ports 18080/18081 forward to
``10.0.2.15:80`` / ``10.0.2.16:80``. DHCP leases can change after restart,
so try both ports. Use ``--port-15`` / ``--port-16`` for concurrent runs.

Manage the same run directory::

  scripts/esp32-sim status "$PWD/run"
  scripts/esp32-sim reset "$PWD/run"
  scripts/esp32-sim stop "$PWD/run"
  scripts/esp32-sim restart "$PWD/run"

Reset resets the guest. Stop quits QEMU. Restart reuses the writable flash,
including guest changes. The helper prints process, QMP socket and log paths.
Set ``QEMU_BIN`` to use a binary built elsewhere.

``--pins /path/to/pins.vcd`` captures resolved digital GPIO pads and drives.
Repeat ``--device 'MODEL,property=value,...'`` to attach QEMU devices; provide
board wiring explicitly. TAS5828M requires ``gpio``, ``sda``, ``scl``,
``bclk``, ``ws`` and ``data`` properties. Optional ``pdn=N`` connects shutdown
to resolved GPIO N; without it, PDN is tied high. Optional ``wav`` names the
audio output. Create output directories before launching.

The default is trace-off ``-icount shift=2,align=off,sleep=off``.
Bluetooth and analog pin behavior are not modeled. ``--help`` lists options.
