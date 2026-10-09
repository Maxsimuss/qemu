#!/usr/bin/env python3
"""Observe TCG instruction execution across selected APLL clock loss/recovery.

The assembly increments a real guest RAM word. MMIO enters through qtest's
physical memory interface while the machine runs TCG. Host waits provide an
observation window only; they do not determine hardware transfer periods.
"""
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parent
QEMU = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('QEMU_BINARY', 'qemu-system-xtensa')
ELF = ROOT / 'build/counter.elf'
LOG = pathlib.Path(os.environ.get('ESP32_CLOCK_GATE_LOG', str(ROOT / 'build/qemu.log')))
# The standard loader enters CPU1 at the ELF entry point. This is fixture
# setup: its runstall/clock gate still follow real guest-visible DPORT MMIO.
entry = int.from_bytes(ELF.read_bytes()[24:28], 'little')

# Keep IPC paths independent of checkout depth (AF_UNIX paths are limited).
with tempfile.TemporaryDirectory(prefix='esp32-apll-') as tmp:
    sockpath = str(pathlib.Path(tmp) / 'qtest.sock')
    with LOG.open('w') as log:
        proc = subprocess.Popen([
            str(QEMU), '-machine', 'esp32', '-accel', 'tcg',
            '-display', 'none', '-serial', 'none', '-nic', 'none',
            '-kernel', str(ELF),
            '-device', f'loader,addr={entry:#x},cpu-num=1',
            '-qtest', f'unix:{sockpath},server=on,wait=off',
            '-qtest-log', '/dev/null',
        ], stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 10
            while not pathlib.Path(sockpath).exists():
                assert proc.poll() is None, 'QEMU exited during startup'
                assert time.monotonic() < deadline, 'QEMU startup timeout'
                time.sleep(0.01)
            sock = socket.socket(socket.AF_UNIX)
            sock.settimeout(5)
            sock.connect(sockpath)
            wire = sock.makefile('rwb', buffering=0)

            def command(cmd):
                wire.write((cmd + '\n').encode())
                reply = wire.readline().decode().strip()
                assert reply.startswith('OK'), (cmd, reply)
                return reply

            def read(address):
                return int(command(f'readl {address:#x}').split()[1], 0)

            def write(address, value):
                command(f'writel {address:#x} {value:#x}')

            def ana_write(reg, value):
                write(0x6000e00c, (1 << 24) | (value << 16) | (reg << 8) | 0x6d)
                deadline = time.monotonic() + 2
                while read(0x6000e00c) & (1 << 25):
                    assert time.monotonic() < deadline, 'bus command timeout'

            def ana_read(reg):
                write(0x6000e00c, (reg << 8) | 0x6d)
                deadline = time.monotonic() + 2
                while True:
                    value = read(0x6000e00c)
                    if not value & (1 << 25):
                        return (value >> 16) & 255
                    assert time.monotonic() < deadline, 'bus command timeout'

            def assert_running(label):
                for core in range(2):
                    address = 0x3ffb0000 + 4 * core
                    first = read(address)
                    deadline = time.monotonic() + 2
                    while read(address) == first:
                        assert time.monotonic() < deadline, f'{label}: CPU{core} did not execute'
                    observed = read(address)
                    time.sleep(0.001)
                    assert read(address) != observed, f'{label}: CPU{core} executed only an in-flight instruction'
                print(label, 'both core counters advance')

            write(0x3ff00030, 1)  # DPORT APPCPU clock enable
            write(0x3ff00034, 0)  # DPORT APPCPU runstall release
            assert_running('XTAL')
            write(0x3ff48000, (read(0x3ff48000) & ~(1 << 18)) | (1 << 19))
            write(0x6000e044, 0x3ff00 & ~(1 << 14))
            write(0x3ff48030, 1 << 24)
            for reg, value in ((4, 4), (5, 0x69), (7, 5), (8, 0), (9, 0),
                               (0, 0x0f), (0, 0x3f), (0, 0x1f)):
                ana_write(reg, value)
            deadline = time.monotonic() + 2
            while not ana_read(3) & 0x80:
                assert time.monotonic() < deadline, 'calibration timeout'
            write(0x3ff48070, 3 << 27)
            assert_running('APLL')
            write(0x3ff48030, 1 << 23)
            # Take the baseline after the MMIO write's main-loop response;
            # an instruction already in flight may finish before the stall.
            time.sleep(0.01)
            baseline = [read(0x3ffb0000 + 4 * core) for core in range(2)]
            for _ in range(20):
                time.sleep(0.005)
                for core in range(2):
                    assert read(0x3ffb0000 + 4 * core) == baseline[core], f'CPU{core} executed with selected clock absent'
            print('APLL power loss: both core counters frozen')
            # External clock mux recovery must release the clock gate.
            write(0x3ff48070, 0)
            assert_running('XTAL recovery')
            # Enter architectural WAITI on both cores. Clock mux recovery
            # must not wake a CPU that was already waiting for interrupt.
            write(0x3ffb0008, 1)
            write(0x3ffb000c, 1)
            time.sleep(0.01)
            sleeping = [read(0x3ffb0000 + 4 * core) for core in range(2)]
            write(0x3ff48070, 3 << 27)  # APLL is still powered down.
            write(0x3ff48070, 0)
            for _ in range(20):
                time.sleep(0.005)
                for core in range(2):
                    assert read(0x3ffb0000 + 4 * core) == sleeping[core], f'CPU{core} WAITI lost on clock recovery'
            print('WAITI state preserved across clock loss and recovery')
            # A guest-visible CPU software reset must leave the processor
            # executable after clock recovery, even after the saved WAITI.
            write(0x3ffb0008, 0)
            write(0x3ffb000c, 0)
            write(0x3ff48000, read(0x3ff48000) | (1 << 5) | (1 << 4))
            assert_running('CPU reset after WAITI')
            # Route independent hardware interrupt inputs to level1 on each core.
            write(0x3ff00104 + 24 * 4, 1)
            write(0x3ff00218 + 25 * 4, 1)
            write(0x3ffb0008, 1)
            write(0x3ffb000c, 1)
            time.sleep(0.01)
            sleeping = [read(0x3ffb0000 + 4 * core) for core in range(2)]
            write(0x3ff48070, 3 << 27)
            command('set_irq_in /machine/soc/intmatrix unnamed-gpio-in 24 1')
            command('set_irq_in /machine/soc/intmatrix unnamed-gpio-in 25 1')
            time.sleep(0.02)
            for core in range(2):
                assert read(0x3ffb0000 + 4 * core) == sleeping[core], f'IRQ woke clock-gated CPU{core}'
            write(0x3ff48070, 0)
            time.sleep(0.01)
            assert read(0x3ffb0008) == 0 and read(0x3ffb000c) == 0, 'pending IRQ handler did not execute after recovery'
            command('set_irq_in /machine/soc/intmatrix unnamed-gpio-in 24 0')
            command('set_irq_in /machine/soc/intmatrix unnamed-gpio-in 25 0')
            assert_running('Pending IRQ on clock recovery')
            print('PASS: TCG clock loss, WAITI, reset and pending interrupt recovery')
            wire.close()
            sock.close()
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
