#!/usr/bin/env python3
"""Boot a real merged ESP32 flash image with the QEMU I2S DAC device."""
import math
import os
from pathlib import Path
import re
import selectors
import shutil
import struct
import subprocess
import sys
import time
import wave

PROJECT = Path(__file__).resolve().parent
ROOT = PROJECT.parents[3]
BUILD = PROJECT / "build"
FLASH_SOURCE = Path(os.environ.get("APLL_DAC_FLASH",
                                   str(BUILD / "flash_image.bin")))
FLASH = BUILD / "run-flash.bin"
WAV = BUILD / "virtual-dac.wav"
LOG = BUILD / "qemu-run.log"
RATE = int(os.environ.get("APLL_DAC_RATE", "16000"))
I2S_PORT = int(os.environ.get("APLL_DAC_I2S_PORT", "0"))
PADS = {0: (18, 19, 23), 1: (5, 25, 26)}
if I2S_PORT not in PADS:
    raise SystemExit("APLL_DAC_I2S_PORT must be 0 or 1")
TONE_FRAMES = RATE * 5
RATE_ID = {16000: 1, 32000: 2}[RATE]
MARKER = (f"VIRTUAL_DAC_PASS frames={TONE_FRAMES} channels=2 bits=16 "
          f"rate={RATE} i2s={I2S_PORT}").encode()


def check_log(output):
    text = output.decode(errors="replace")
    expected = [
        rf"VIRTUAL_DAC_BEGIN frames={TONE_FRAMES} rate={RATE} channels=2 bits=16 i2s={I2S_PORT}",
        r"DAC_STATUS phase=idle value=([0-9a-f]{2})",
        r"DAC_WRITE reg=00 value=02",
        r"DAC_WRITE reg=01 value=10",
        rf"DAC_WRITE reg=02 value={RATE_ID:02x}",
        r"DAC_STATUS phase=idle value=([0-9a-f]{2})",
        r"APLL_CHECK cal_end=1 odiv=\d+ sdm=\d+:\d+:\d+",
        r"DAC_WRITE reg=03 value=01",
        r"DAC_STATUS phase=recording value=([0-9a-f]{2})",
        r"DAC_STATUS phase=recording value=([0-9a-f]{2})",
        r"DAC_WRITE reg=03 value=00",
        r"DAC_STATUS phase=idle value=([0-9a-f]{2})",
        rf"VIRTUAL_DAC_PASS frames={TONE_FRAMES} channels=2 bits=16 rate={RATE} i2s={I2S_PORT}",
    ]
    position = 0
    statuses = []
    for item in expected:
        match = re.search(item, text[position:])
        if not match:
            raise AssertionError(f"Firmware marker missing or out of order: {item}; see {LOG}")
        if match.groups():
            statuses.append(int(match[1], 16))
        position += match.end()
    if any(status & 3 for status in statuses):
        raise AssertionError(f"Clock/config errors in DAC status: {statuses}")
    if [bool(status & 4) for status in statuses] != [False, False, True, True, False]:
        raise AssertionError(f"Wrong recording state: {statuses}")
    return statuses


def check_wav():
    with wave.open(str(WAV), "rb") as source:
        channels = source.getnchannels()
        width = source.getsampwidth()
        rate = source.getframerate()
        frames = source.getnframes()
        assert (channels, width, rate) == (2, 2, RATE), (channels, width, rate)
        assert TONE_FRAMES <= frames <= TONE_FRAMES + 2 * RATE, \
            f"Capture does not contain the full transmitted tone: {frames} frames"
        pcm = source.readframes(frames)
    assert len(pcm) == frames * 4, f"Short PCM payload: {len(pcm)}"
    samples = struct.unpack(f"<{frames * 2}h", pcm)
    left, right = samples[::2], samples[1::2]
    assert left == right, "Stereo channels disagree"
    if RATE == 32000:
        cycle = (0, 4520, 8867, 12846, 16384, 19308, 21407, 22725,
                 23170, 22725, 21407, 19308, 16384, 12846, 8867, 4520,
                 0, -4520, -8867, -12846, -16384, -19308, -21407, -22725,
                 -23170, -22725, -21407, -19308, -16384, -12846, -8867, -4520)
    else:
        cycle = (0, 8867, 16384, 21407, 23170, 21407, 16384, 8867,
                 0, -8867, -16384, -21407, -23170, -21407, -16384, -8867)
    starts = [i for i in range(len(left) - TONE_FRAMES + 1)
              if left[i] == 0 and left[i + 1] == cycle[1]]
    tone_start = next((i for i in starts if all(
        left[i + j] == cycle[j % len(cycle)] for j in range(TONE_FRAMES))), None)
    assert tone_start is not None, "WAV is missing the complete transmitted PCM sequence"
    assert all(value == 0 for value in left[:tone_start]), "Unexpected PCM before the tone"
    assert all(value == 0 for value in left[tone_start + TONE_FRAMES:]), \
        "Unexpected PCM after the tone"
    tone = left[tone_start:tone_start + TONE_FRAMES]
    peak = max(abs(n) for n in tone)
    rms = math.sqrt(sum(n * n for n in tone) / len(tone))
    expected_peak = 32767 * math.sqrt(0.5)
    expected_rms = expected_peak / math.sqrt(2)
    assert abs(peak - expected_peak) < expected_peak * 0.05, f"Peak amplitude: {peak}"
    assert abs(rms - expected_rms) < expected_rms * 0.05, f"RMS amplitude: {rms}"
    rising = sum(a <= 0 < b for a, b in zip(tone, tone[1:]))
    frequency = rising * rate / len(tone)
    assert abs(frequency - 1000) <= 10, f"Signal frequency: {frequency} Hz"
    return frames, peak, rms, frequency


def main():
    if not FLASH_SOURCE.is_file():
        raise SystemExit("Build merged flash first: ./build.sh")
    optimized = ROOT / "build/qemu-audio/qemu-system-xtensa"
    default_qemu = optimized if optimized.is_file() else ROOT / "build/qemu/qemu-system-xtensa"
    qemu = Path(os.environ.get("QEMU_BIN", default_qemu))
    if not qemu.is_file():
        raise SystemExit(f"QEMU executable not present: {qemu}")
    shutil.copyfile(FLASH_SOURCE, FLASH)
    WAV.unlink(missing_ok=True)
    # Fixed 128ns instruction quanta are smaller than the ~977ns BCLK half
    # period and yield <0.5% frame-period quantization at 16kHz. All peripheral
    # edges still traverse the real timers and resolved pad listeners.
    # Adaptive icount is a CPU-speed controller, not edge-adaptive precision.
    # Use shift=0 separately when checking individual nanosecond edge times.
    shift = os.environ.get("ICOUNT_SHIFT", "7")
    # Instruction counting requires single-threaded TCG; retain deterministic
    # CPU ordering while peripheral timers use their original virtual deadlines.
    cmd = [str(qemu), "-machine", "esp32", "-accel", "tcg,thread=single",
           "-display", "none", "-monitor", "none",
           "-serial", "stdio", "-nic", "none", "-icount", f"shift={shift},align=off,sleep=off",
           "-drive", f"file={FLASH},if=mtd,format=raw",
           "-device", f"esp32-i2s-dac,gpio=/machine/soc/gpio,wav={WAV},"
                      f"bclk={PADS[I2S_PORT][0]},ws={PADS[I2S_PORT][1]},"
                      f"data={PADS[I2S_PORT][2]}"]
    print("QEMU:", " ".join(cmd), flush=True)
    output = bytearray()
    started_wall = time.monotonic()
    boot_wall = capture_begin_wall = capture_end_wall = None
    deadline = started_wall + float(os.environ.get("DAC_TIMEOUT", "1200"))
    with subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT) as process:
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            while time.monotonic() < deadline:
                for key, _ in selector.select(timeout=1):
                    chunk = os.read(key.fd, 65536)
                    output.extend(chunk)
                    LOG.write_bytes(output)
                    now = time.monotonic()
                    if boot_wall is None and b"VIRTUAL_DAC_BEGIN" in output:
                        boot_wall = now - started_wall
                    if capture_begin_wall is None and b"DAC_WRITE reg=03 value=01" in output:
                        capture_begin_wall = now
                    if (capture_begin_wall is not None and capture_end_wall is None and
                            b"DAC_WRITE reg=03 value=00" in output):
                        capture_end_wall = now
                if (MARKER in output or b"VIRTUAL_DAC_FAIL" in output or
                        b"abort()" in output or process.poll() is not None):
                    break
            else:
                raise TimeoutError(f"QEMU exceeded {os.environ.get('DAC_TIMEOUT', '1200')} seconds")
        finally:
            selector.close()
            process.terminate() if process.poll() is None else None
            try:
                tail, _ = process.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                tail, _ = process.communicate()
            output.extend(tail)
            LOG.write_bytes(output)
    if MARKER not in output:
        raise AssertionError(f"Firmware did not pass; see {LOG}:\n{output[-5000:].decode(errors='replace')}")
    statuses = check_log(output)
    frames, peak, rms, frequency = check_wav()
    counters = re.findall(rb"DAC_COUNTER phase=idle frames=(\d+)", output)
    assert counters and int(counters[-1]) == frames, "I2C frame counter and WAV disagree"
    wall = time.monotonic() - started_wall
    capture_wall = (capture_end_wall - capture_begin_wall
                    if capture_end_wall is not None else None)
    assert capture_wall is not None and capture_wall > 0, "No measured capture interval"
    speed = 5.0 / capture_wall
    print(f"VIRTUAL_DAC_WAVEFORM_PASS frames={frames} tone_frames={TONE_FRAMES} "
          f"rate={RATE} duration={frames / RATE:.4f}s "
          f"peak={peak} rms={rms:.1f} frequency={frequency:.2f}Hz statuses={statuses} "
          f"wall={wall:.3f}s boot_wall={boot_wall:.3f}s capture_wall={capture_wall:.3f}s "
          f"realtime_factor={speed:.3f} wall_per_audio_second={wall / (frames / 16000):.3f}")
    assert speed >= 0.2, f"Capture below minimum 0.2x realtime: {speed:.3f}x"
    # Atomically promote only a complete, correct, sufficiently fast capture.
    artifact = BUILD / "firmware-audio.wav"
    temporary = BUILD / "firmware-audio.next.wav"
    shutil.copyfile(WAV, temporary)
    os.replace(temporary, artifact)
    print(f"VIRTUAL_DAC_QEMU_PASS verified_audio={artifact}")


if __name__ == "__main__":
    main()
