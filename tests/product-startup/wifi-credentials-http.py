#!/usr/bin/env python3
"""Drive the supplied MODUAMP HTTP credential endpoint through guest Wi-Fi."""

import argparse
import hashlib
import http.client
import json
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time


def request(port, method, path, body=None, timeout=4):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=timeout)
    try:
        conn.request(method, path, body=body,
                     headers={"Content-Type": "application/json"}
                     if body is not None else {})
        response = conn.getresponse()
        payload = response.read()
        return response.status, payload
    finally:
        conn.close()


def qmp_receive_reply(sock):
    pending = bytearray()
    while True:
        pending.extend(sock.recv(4096))
        while b"\n" in pending:
            line, _, rest = pending.partition(b"\n")
            pending[:] = rest
            if not line.strip():
                continue
            message = json.loads(line)
            if "return" in message or "error" in message:
                return message


def qmp_guest_clock_skew(qmp_path):
    """Return QEMU's host-minus-guest clock drift from TCG's QMP stats."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as qmp:
        qmp.settimeout(5)
        qmp.connect(str(qmp_path))
        greeting = json.loads(qmp.recv(4096).splitlines()[0])
        if "QMP" not in greeting:
            raise RuntimeError(f"unexpected QMP greeting: {greeting!r}")
        qmp.sendall(b'{"execute":"qmp_capabilities"}\r\n')
        if "error" in qmp_receive_reply(qmp):
            raise RuntimeError("QMP capability negotiation failed")
        qmp.sendall(b'{"execute":"x-query-jit"}\r\n')
        reply = qmp_receive_reply(qmp)
        if "error" in reply:
            raise RuntimeError(f"QMP x-query-jit failed: {reply['error']}")
        report = reply["return"].get("human-readable-text", "")
        match = re.search(r"Host - Guest clock\s+(-?\d+) ms", report)
        if not match:
            raise RuntimeError(f"QMP TCG clock drift missing from {report!r}")
        return int(match.group(1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qemu", required=True, help="qemu-system-xtensa path")
    parser.add_argument("--image", required=True,
                        help="pristine original merged ESP32 flash image")
    parser.add_argument("--port", type=int, default=18080,
                        help="host TCP port forwarded via SLIRP's 10.0.2.15 guest endpoint")
    parser.add_argument("--sta-port", type=int,
                        help="host TCP port forwarded to the SLIRP STA lease (default: AP port + 1)")
    parser.add_argument("--ssid", default="test-ap")
    parser.add_argument("--password", default="test-password")
    parser.add_argument("--deadline", type=float, default=600,
                        help="seconds to wait for guest HTTP server")
    parser.add_argument("--sta-deadline", type=float, default=60,
                        help="seconds to wait for STA HTTP reachability after credential POST")
    parser.add_argument("--reboot-deadline", type=float, default=600,
                        help="seconds to wait for HTTP service after QMP reboot")
    parser.add_argument("--gdb-port", type=int,
                        help="optional Xtensa GDB port for stage breakpoints")
    parser.add_argument("--trace-off", action="store_true",
                        help="disable QEMU tracing, pcap capture and guest-error logging")
    parser.add_argument("--measure-speed", action="store_true",
                        help="measure guest virtual-time rate through QMP; implies --trace-off")
    parser.add_argument("--log-dir", default="tests/product-startup/run")
    args = parser.parse_args()
    if args.measure_speed and args.gdb_port:
        parser.error("--measure-speed cannot be combined with GDB")

    image = Path(args.image).resolve()
    qemu = Path(args.qemu).resolve()
    if not image.is_file() or image.stat().st_size != 4 * 1024 * 1024:
        parser.error("--image must be the verified 4 MiB original merged flash")
    if not qemu.is_file():
        parser.error("--qemu does not exist")

    log_dir = Path(args.log_dir).resolve()
    sta_port = args.sta_port or args.port + 1
    log_dir.mkdir(parents=True, exist_ok=True)
    # QEMU writes NVS into the raw flash. Always start from a fresh per-run
    # copy so this test preserves the factory fixture and reboot checks use
    # only state persisted by this run.
    working_image = log_dir / "working-flash.bin"
    if working_image == image:
        parser.error("working flash image must differ from the pristine input")
    shutil.copyfile(image, working_image)
    source_hash = hashlib.sha256(image.read_bytes()).hexdigest()
    initial_work_hash = hashlib.sha256(working_image.read_bytes()).hexdigest()
    (log_dir / "flash-manifest.txt").write_text(
        f"source={image}\nsource_sha256={source_hash}\n"
        f"working_copy={working_image}\n"
        f"working_initial_sha256={initial_work_hash}\n"
        "qemu_writes_nvs_to_working_copy=true\n")
    qemu_log = log_dir / "qemu.log"
    qemu_stderr = log_dir / "qemu.stderr"
    serial_log = log_dir / "serial.log"
    pcap_file = log_dir / "slirp.pcap"
    trace_file = log_dir / "wifi.trace"
    trace_events = log_dir / "wifi-trace-events"
    qmp_socket = log_dir / "qmp.sock"
    gdb_log = log_dir / "gdb.log"
    trace_events.write_text("esp32_wifi_guest_frame\nesp32_wifi_rx_dma_attempt\n"
                            "esp32_wifi_rx_dma_complete\n"
                            "esp32_wifi_softap_dhcp\n"
                            "esp32_wifi_softap_dhcp_offer\n"
                            "esp32_wifi_softap_dhcp_request\n"
                            "esp32_wifi_backend_receive\n"
                            "esp32_wifi_slirp_arp_proxy\n"
                            "esp32_wifi_softap_uplink\n"
                            "esp32_wifi_softap_tx_frame\n"
                            "esp32_wifi_softap_arp\n")
    qmp_socket.unlink(missing_ok=True)
    machine = ("esp32,wifi-peer-ssid=" + args.ssid +
               ",wifi-peer-password=" + args.password)
    # The AP-side virtual client gets 10.0.2.15 first; the firmware STA is
    # a second DHCP client and gets 10.0.2.16 on this explicit SLIRP subnet.
    command = [
        str(qemu), "-L", str(Path(__file__).resolve().parents[2] / "pc-bios"),
        "-machine", machine, "-m", "4M", "-display", "none",
        "-serial", "null" if (args.trace_off or args.measure_speed) else
                   "file:" + str(serial_log),
        "-drive", f"file={working_image},if=mtd,format=raw",
        # SLIRP can forward only to its own virtual guest address here. The
        # Wi-Fi backend then translates that packet onto the observed AP
        # subnet (192.168.4.1 plus the real DHCP lease) after DHCP completes.
        # The second host port uses the same SLIRP endpoint in STA mode, where
        # the Wi-Fi backend passes the packet without AP address translation.
        "-nic", ("user,id=wifi-slirp,model=misc.esp32_wifi,net=10.0.2.0/24,"
                 "dhcpstart=10.0.2.15,hostfwd=tcp::" + str(args.port) +
                 "-10.0.2.15:80,hostfwd=tcp::" + str(sta_port) +
                 "-10.0.2.16:80"),
        "-qmp", f"unix:{qmp_socket},server=on,wait=off",
        "-device", "esp32-tas5828m,gpio=/machine/soc/gpio,address=0x61,"
                   "driver-slot=2,bclk=5,ws=17,data=18",
        "-device", "esp32-tas5828m,gpio=/machine/soc/gpio,address=0x62,"
                   "driver-slot=3,bclk=5,ws=17,data=18",
        "-icount", "shift=2,align=off,sleep=off",
    ]
    trace_off = args.trace_off or args.measure_speed
    if not trace_off:
        command.extend([
            "-object", f"filter-dump,id=wifi-pcap,netdev=wifi-slirp,file={pcap_file}",
            "-trace", f"events={trace_events},file={trace_file}",
            "-d", "guest_errors", "-D", str(qemu_log),
        ])
    if args.gdb_port:
        command.extend(["-S", "-gdb", f"tcp::{args.gdb_port}"])

    # Fail early if the chosen host-forward port is already occupied.
    for port in {args.port, sta_port}:
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as exc:
                parser.error(f"host port {port} is unavailable: {exc}")

    print("Starting original firmware; waiting for its SoftAP HTTP server.",
          flush=True)
    proc = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                            stdout=subprocess.DEVNULL,
                            stderr=qemu_stderr.open("w"))
    gdb_proc = None
    try:
        if args.gdb_port:
            gdb_script = log_dir / "stages.gdb"
            stages = [
                ("init_i2s", 0x400dca80),
                ("init_wifi", 0x400dc630),
                ("esp_wifi_init", 0x400fa3f4),
                ("esp_wifi_start", 0x4012ffa8),
                ("start_webserver", 0x400e10e0),
            ]
            lines = ["set pagination off", "set confirm off",
                     f"target remote :{args.gdb_port}"]
            for name, address in stages:
                lines.extend([
                    f"break *{address:#x}",
                    "commands",
                    "silent",
                    f'printf "MILESTONE {name} pc=%x ccount=%x\\n", $pc, $ccount',
                    "continue",
                    "end",
                ])
            lines.extend(["continue", "detach", "quit"])
            gdb_script.write_text("\n".join(lines) + "\n")
            gdb_candidates = [
                Path("/workspace/esp32-sim/tools/espressif/tools/xtensa-esp-elf-gdb/17.1_20260402/xtensa-esp-elf-gdb/bin/xtensa-esp32-elf-gdb"),
            ]
            gdb = next((candidate for candidate in gdb_candidates
                        if candidate.is_file()), None)
            if gdb is None:
                raise RuntimeError("--gdb-port requested but workspace Xtensa GDB is missing")
            gdb_proc = subprocess.Popen(
                [str(gdb), "-q", "-batch", "-x", str(gdb_script)],
                stdin=subprocess.DEVNULL, stdout=gdb_log.open("w"),
                stderr=subprocess.STDOUT)
        deadline = time.monotonic() + args.deadline
        speed_start_wall = speed_start_skew = None
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"QEMU exited early with status {proc.returncode}; "
                                   f"see {qemu_log}")
            try:
                status, _ = request(args.port, "GET", "/", timeout=2)
                if status == 200:
                    if args.measure_speed:
                        speed_start_wall = time.monotonic()
                        speed_start_skew = qmp_guest_clock_skew(qmp_socket)
                    break
            except (OSError, TimeoutError):
                time.sleep(0.5)
        else:
            raise TimeoutError(f"guest HTTP server not reached in {args.deadline}s; "
                               f"see {qemu_log} and {serial_log}")

        payload = json.dumps({"ssid": args.ssid,
                              "password": args.password},
                             separators=(",", ":"))
        post_error = None
        try:
            status, body = request(args.port, "POST", "/api/wifi_credentials",
                                   payload)
            if not 200 <= status < 300:
                raise RuntimeError(f"credential POST returned HTTP {status}: "
                                   f"{body[:256]!r}")
            print(f"Guest returned HTTP {status} to POST /api/wifi_credentials.")
        except (OSError, TimeoutError) as exc:
            # The supplied handler stops SoftAP while applying the new STA
            # configuration, which can close the HTTP connection before its
            # response reaches this client.  The STA endpoint below is the
            # evidence that the request was consumed and the credentials
            # were applied.
            post_error = str(exc)
            print("POST connection ended during the Wi-Fi mode change; "
                  "waiting for the STA endpoint.", flush=True)
        print("Waiting for the separate STA host-forward after WPA2/DHCP.",
              flush=True)
        deadline = time.monotonic() + args.sta_deadline
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"QEMU exited before STA HTTP with status {proc.returncode}; "
                                   f"see {qemu_log}")
            try:
                status, _ = request(sta_port, "GET", "/", timeout=2)
                if status == 200:
                    break
            except (OSError, TimeoutError):
                time.sleep(0.5)
        else:
            detail = f" (POST response: {post_error})" if post_error else ""
            raise TimeoutError(f"STA HTTP server not reached in {args.sta_deadline}s"
                               f"{detail}; see {qemu_log}, {pcap_file}, and "
                               f"{serial_log}")
        print(f"Guest returned HTTP 200 through the STA forward on port {sta_port}.",
              flush=True)
        if proc.poll() is not None:
            raise RuntimeError(f"QEMU exited before reboot with status {proc.returncode}; "
                               f"see {qemu_log}")

        # Issue an actual QMP system reset so the next boot must reload the
        # credentials that the HTTP handler stored in NVS.
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as qmp:
            qmp.settimeout(5)
            qmp.connect(str(qmp_socket))
            greeting = qmp.recv(4096)
            if b"QMP" not in greeting:
                raise RuntimeError(f"unexpected QMP greeting: {greeting!r}")
            qmp.sendall(b'{"execute":"qmp_capabilities"}\r\n')
            if b"return" not in qmp.recv(4096):
                raise RuntimeError("QMP capability negotiation failed")
            qmp.sendall(b'{"execute":"system_reset"}\r\n')
            response = qmp.recv(4096)
            if b"return" not in response and b"RESET" not in response:
                raise RuntimeError(f"QMP reset was not accepted: {response!r}")
        print("Waiting for HTTP service after reboot.", flush=True)
        deadline = time.monotonic() + args.reboot_deadline
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"QEMU exited after reboot with status {proc.returncode}; "
                                   f"see {qemu_log}")
            try:
                status, _ = request(sta_port, "GET", "/", timeout=2)
                if status == 200:
                    break
            except (OSError, TimeoutError):
                time.sleep(1)
        else:
            raise TimeoutError(f"no HTTP 200 after reboot in "
                               f"{args.reboot_deadline}s; see {qemu_log} and "
                               f"{serial_log}")
        print(f"Guest returned HTTP 200 after reboot; logs: {log_dir}")
        if args.measure_speed:
            speed_end_skew = qmp_guest_clock_skew(qmp_socket)
            wall_seconds = time.monotonic() - speed_start_wall
            guest_seconds = (wall_seconds * 1000 -
                             (speed_end_skew - speed_start_skew)) / 1000
            factor = guest_seconds / wall_seconds if wall_seconds > 0 else 0
            print(f"TRACE_OFF_SPEED guest_seconds={guest_seconds:.3f} "
                  f"wall_seconds={wall_seconds:.3f} realtime_factor={factor:.3f}")
            if factor < 0.2:
                raise RuntimeError(f"trace-off guest speed {factor:.3f}x is below 0.2x")
        print("This proves HTTP service reachability only. Confirm WPA2 association, "
              "DHCP lease, and STA-vs-fallback mode from guest-side frame evidence.")
        return 0
    except Exception as exc:
        print(f"acceptance step failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if gdb_proc is not None and gdb_proc.poll() is None:
            gdb_proc.terminate()
            try:
                gdb_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                gdb_proc.kill()
                gdb_proc.wait()
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


if __name__ == "__main__":
    raise SystemExit(main())
