#!/usr/bin/env python3
import argparse
import asyncio
import hashlib
import json
import socket
import sys
import threading
import time
import urllib.request
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable

import serial_asyncio

import diy_ffb_protocol_pb2 as ffb_protocol
from serial_test import OutputProtocol as BaseOutputProtocol

class OutputProtocol(BaseOutputProtocol):
    def connection_lost(self, exc):
        print("port closed")


OTA_TARGETS = {
    "all": ffb_protocol.OTA_TARGET_ALL,
    "axes": ffb_protocol.OTA_TARGET_AXES_ONLY,
    "gateway": ffb_protocol.OTA_TARGET_GATEWAY_ONLY,
}
MAX_AXES = 8
GATEWAY_IDS = [ffb_protocol.GATEWAY_ID_1, ffb_protocol.GATEWAY_ID_2]


def resolve_host_ip(host: str, bind: str) -> str:
    if host:
        return host
    if bind and bind not in ("0.0.0.0", "::"):
        return bind
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return "127.0.0.1"


def make_handler(json_bytes: bytes, firmware_bytes: bytes, verbose: bool):
    class OtaHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/update_info.json"):
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(json_bytes)))
                self.end_headers()
                self.wfile.write(json_bytes)
                return
            if self.path == "/firmware.bin":
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(firmware_bytes)))
                self.end_headers()
                self.wfile.write(firmware_bytes)
                return
            self.send_error(404, "not found")

        def log_message(self, fmt, *args):
            if verbose:
                super().log_message(fmt, *args)

    return OtaHandler


def compute_md5_bytes(data: bytes) -> str:
    md5 = hashlib.md5()
    md5.update(data)
    return md5.hexdigest()


def start_ota_server(firmware_bytes: bytes, board: str, version: str, md5_hex: str, bind: str, host: str, port: int, verbose: bool):
    host_ip = resolve_host_ip(host, bind)
    firmware_url = f"http://{host_ip}:{port}/firmware.bin"
    info_payload = {"Configurations": [{"Board": board, "Version": version, "URL": firmware_url, "MD5": md5_hex}]}
    json_bytes = json.dumps(info_payload).encode("ascii")
    handler = make_handler(json_bytes, firmware_bytes, verbose)
    server = ThreadingHTTPServer((bind, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    info_url = f"http://{host_ip}:{port}/update_info.json"
    return server, thread, info_url


def log_message(msg, verbose: bool) -> None:
    payload_type = msg.WhichOneof("payload")
    if payload_type == "axis_log_message":
        log_msg = msg.axis_log_message
        print(f"{ffb_protocol.AxisID.Name(log_msg.axis_id)} : {log_msg.msg.rstrip()}")
    elif payload_type == "gateway_log_message":
        log_msg = msg.gateway_log_message
        print(f"{ffb_protocol.GatewayID.Name(log_msg.gateway_id)} : {log_msg.msg.rstrip()}")
    elif verbose:
        print(f"unhandled message received: {msg}")


def format_device_info(info: ffb_protocol.DeviceInfo) -> str:
    parts: list[str] = []
    if info.fw_version:
        parts.append(f"fw={info.fw_version}")
    if info.board:
        parts.append(f"board={info.board}")
    if info.git_hash:
        parts.append(f"git={info.git_hash}")
    if info.device_uid:
        parts.append(f"uid={info.device_uid}")
    if info.build_timestamp:
        parts.append(f"built={info.build_timestamp}")
    return " ".join(parts) if parts else "no details"


def print_device_info_summary(
    axis_ids: list[int],
    axis_info: dict[int, ffb_protocol.DeviceInfo],
    gateway_ids: list[int],
    gateway_info: dict[int, ffb_protocol.DeviceInfo],
) -> None:
    print("Device info at startup:")
    for axis_id in axis_ids:
        axis_name = ffb_protocol.AxisID.Name(axis_id)
        info = axis_info.get(axis_id)
        if info is None:
            print(f"{axis_name} : no response")
        else:
            print(f"{axis_name} : {format_device_info(info)}")
    for gateway_id in gateway_ids:
        gateway_name = ffb_protocol.GatewayID.Name(gateway_id)
        info = gateway_info.get(gateway_id)
        if info is None:
            print(f"{gateway_name} : no response")
        else:
            print(f"{gateway_name} : {format_device_info(info)}")


def build_axis_start_ota_message(args, info_url: str, axis_id: int) -> ffb_protocol.Message:
    msg = ffb_protocol.Message()
    msg.start_ota_update.wifi_info.ssid = args.ssid
    msg.start_ota_update.wifi_info.password = args.password
    msg.start_ota_update.info_json_url = info_url
    msg.start_ota_update.allow_downgrades = args.allow_downgrades
    msg.start_ota_update.target = OTA_TARGETS["axes"]
    msg.start_ota_update.target_axis_id = axis_id
    return msg


def build_gateway_start_ota_message(args, info_url: str) -> ffb_protocol.Message:
    msg = ffb_protocol.Message()
    msg.start_ota_update.wifi_info.ssid = args.ssid
    msg.start_ota_update.wifi_info.password = args.password
    msg.start_ota_update.info_json_url = info_url
    msg.start_ota_update.allow_downgrades = args.allow_downgrades
    msg.start_ota_update.target = OTA_TARGETS["gateway"]
    return msg


def axis_id_from_number(axis_number: int) -> int:
    if axis_number < 1 or axis_number > MAX_AXES:
        raise ValueError(f"axis must be in range 1..{MAX_AXES}")
    return getattr(ffb_protocol, f"AXIS_ID_{axis_number}")


async def request_device_info(
    protocol: OutputProtocol,
    axis_ids: list[int],
    gateway_ids: list[int],
    timeout: float,
    verbose: bool,
) -> tuple[dict[int, ffb_protocol.DeviceInfo], dict[int, ffb_protocol.DeviceInfo]]:
    if not axis_ids and not gateway_ids:
        return {}, {}
    for axis_id in axis_ids:
        msg = ffb_protocol.Message()
        msg.device_info_request.axis_id = axis_id
        protocol.send_message(msg)
        await asyncio.sleep(0.05)
    for gateway_id in gateway_ids:
        msg = ffb_protocol.Message()
        msg.device_info_request.gateway_id = gateway_id
        protocol.send_message(msg)
        await asyncio.sleep(0.05)

    deadline = time.time() + timeout
    axis_results: dict[int, ffb_protocol.DeviceInfo] = {}
    gateway_results: dict[int, ffb_protocol.DeviceInfo] = {}
    while time.time() < deadline and (len(axis_results) < len(axis_ids) or len(gateway_results) < len(gateway_ids)):
        timeout_left = max(0.1, deadline - time.time())
        try:
            msg = await asyncio.wait_for(protocol.msg_queue.get(), timeout_left)
        except asyncio.TimeoutError:
            continue
        payload_type = msg.WhichOneof("payload")
        if payload_type == "device_info":
            source_type = msg.device_info.WhichOneof("source")
            if source_type == "axis_id" and msg.device_info.axis_id in axis_ids:
                axis_results[msg.device_info.axis_id] = msg.device_info
                if verbose:
                    axis_name = ffb_protocol.AxisID.Name(msg.device_info.axis_id)
                    print(f"{axis_name} : DeviceInfo fw={msg.device_info.fw_version}")
            elif source_type == "gateway_id" and msg.device_info.gateway_id in gateway_ids:
                gateway_results[msg.device_info.gateway_id] = msg.device_info
                if verbose:
                    gateway_name = ffb_protocol.GatewayID.Name(msg.device_info.gateway_id)
                    print(f"{gateway_name} : DeviceInfo fw={msg.device_info.fw_version}")
        else:
            log_message(msg, verbose)
    return axis_results, gateway_results


def load_ffbota(path: Path) -> tuple[dict, bytes]:
    with zipfile.ZipFile(path, "r") as archive:
        try:
            manifest_bytes = archive.read("manifest.json")
        except KeyError as exc:
            raise ValueError("ffbota missing manifest.json") from exc
        try:
            firmware_bytes = archive.read("firmware.bin")
        except KeyError as exc:
            raise ValueError("ffbota missing firmware.bin") from exc
    manifest = json.loads(manifest_bytes.decode("utf-8"))
    return manifest, firmware_bytes


def fetch_update_info(url: str) -> dict:
    with urllib.request.urlopen(url) as response:
        payload = response.read().decode("utf-8")
    return json.loads(payload)


def resolve_expected_version(info_payload: dict | None) -> str:
    if info_payload:
        configs = info_payload.get("Configurations", [])
        if configs:
            version = configs[0].get("Version", "")
            return version.strip() if isinstance(version, str) else ""
    return ""


def _versions_match(actual: str, expected: str) -> bool:
    return actual.strip().lower() == expected.strip().lower()


async def update_target_group(
    protocol: OutputProtocol,
    target_ids: list[int],
    label: str,
    name_for: Callable[[int], str],
    expected_version: str,
    build_update_msg: Callable[[int], ffb_protocol.Message],
    build_info_request: Callable[[int], ffb_protocol.Message],
    is_my_device_info: Callable[[ffb_protocol.Message, int], bool],
    retry_count: int,
    retry_timeout: float,
    poll_interval: float,
    verbose: bool,
) -> bool:
    """Send StartOtaUpdate to each target, then poll DeviceInfo until all targets
    report `expected_version` or `retry_timeout` elapses. Retries up to `retry_count`
    additional times. Mirrors SimHub `OtaUpdateCoordinator` semantics."""
    if not target_ids:
        return True

    if not expected_version or expected_version == "-":
        print(f"OTA: skipping {label} version checks (no expected version).")
        for tid in target_ids:
            protocol.send_message(build_update_msg(tid))
            await asyncio.sleep(0.02)
        return True

    for attempt in range(retry_count + 1):
        print(f"OTA: sending {label} update (attempt {attempt + 1}/{retry_count + 1}).")
        for tid in target_ids:
            protocol.send_message(build_update_msg(tid))
            await asyncio.sleep(0.02)

        versions: dict[int, str] = {}
        deadline = time.time() + retry_timeout
        next_poll = 0.0
        while time.time() < deadline:
            now = time.time()
            if now >= next_poll:
                for tid in target_ids:
                    if not _versions_match(versions.get(tid, ""), expected_version):
                        protocol.send_message(build_info_request(tid))
                        await asyncio.sleep(0.02)
                next_poll = now + poll_interval

            wait_for = max(0.05, min(deadline, next_poll) - time.time())
            try:
                msg = await asyncio.wait_for(protocol.msg_queue.get(), wait_for)
            except asyncio.TimeoutError:
                continue

            if msg.WhichOneof("payload") == "device_info":
                for tid in target_ids:
                    if is_my_device_info(msg, tid):
                        versions[tid] = msg.device_info.fw_version.strip()
                        if verbose:
                            print(f"{name_for(tid)} : DeviceInfo fw={versions[tid]}")
                        break
            else:
                log_message(msg, verbose)

            if all(_versions_match(versions.get(tid, ""), expected_version) for tid in target_ids):
                print(f"OTA: all {label} targets at expected version.")
                return True

        failed = [tid for tid in target_ids if not _versions_match(versions.get(tid, ""), expected_version)]
        print(f"OTA: {label} attempt {attempt + 1} did not converge ({len(failed)} pending):")
        for tid in failed:
            current = versions.get(tid) or "<no response>"
            print(f"  -> {name_for(tid)} (current: {current})")

    return False


async def main_async(args) -> int:
    server = None
    thread = None
    info_url = args.url
    info_payload = None
    if args.firmware:
        firmware_path = Path(args.firmware).expanduser()
        if not firmware_path.exists():
            raise FileNotFoundError(f"firmware not found: {firmware_path}")
        if firmware_path.suffix.lower() != ".ffbota":
            raise ValueError("--firmware expects a .ffbota container")
        manifest, firmware_bytes = load_ffbota(firmware_path)
        version = str(manifest.get("version", "")).strip()
        board = str(manifest.get("board", "")).strip()
        md5_hex = str(manifest.get("md5", "")).strip()
        if not md5_hex:
            raise ValueError("ffbota manifest.json missing md5")
        computed_md5 = compute_md5_bytes(firmware_bytes)
        if computed_md5.lower() != md5_hex.lower():
            raise ValueError(f"ffbota MD5 mismatch: manifest {md5_hex} vs computed {computed_md5}")
        info_payload = {"Configurations": [{"Board": board or "unknown", "Version": version, "URL": "", "MD5": md5_hex}]}
        server, thread, info_url = start_ota_server(
            firmware_bytes=firmware_bytes,
            board=board or "unknown",
            version=version,
            md5_hex=md5_hex,
            bind=args.bind,
            host=args.host,
            port=args.serve_port,
            verbose=args.verbose,
        )
        print(f"Serving OTA at {info_url}")
    elif args.url:
        info_payload = fetch_update_info(args.url)

    expected_version = resolve_expected_version(info_payload)

    loop = asyncio.get_running_loop()
    transport, protocol = await serial_asyncio.create_serial_connection(loop, OutputProtocol, args.port, baudrate=args.baud)
    await asyncio.sleep(0.1)

    assessment_axis_ids = [axis_id_from_number(idx) for idx in range(1, MAX_AXES + 1)]
    axis_info, gateway_info = await request_device_info(
        protocol,
        assessment_axis_ids,
        GATEWAY_IDS,
        args.device_info_timeout,
        args.verbose,
    )
    print_device_info_summary(assessment_axis_ids, axis_info, GATEWAY_IDS, gateway_info)
    present_axes = [axis_id for axis_id in assessment_axis_ids if axis_id in axis_info]
    present_axis_set = set(present_axes)
    requested_axis_ids = [axis_id_from_number(args.axis)] if args.axis else assessment_axis_ids
    axis_ids = [axis_id for axis_id in requested_axis_ids if axis_id in present_axis_set]
    missing_axis_ids = [axis_id for axis_id in requested_axis_ids if axis_id not in present_axis_set]
    if missing_axis_ids:
        print("Skipping axes with no device info response:")
        for axis_id in missing_axis_ids:
            axis_name = ffb_protocol.AxisID.Name(axis_id)
            print(f" -> {axis_name}")

    present_gateways = [gid for gid in GATEWAY_IDS if gid in gateway_info]
    if args.target == "gateway" and args.axis is not None:
        raise ValueError("--axis cannot be used with --target gateway")

    axes_targeted = args.target in ("all", "axes")
    gateway_targeted = args.target in ("all", "gateway")

    def axis_info_request(axis_id: int) -> ffb_protocol.Message:
        m = ffb_protocol.Message()
        m.device_info_request.axis_id = axis_id
        return m

    def axis_matches(msg: ffb_protocol.Message, axis_id: int) -> bool:
        return (
            msg.device_info.WhichOneof("source") == "axis_id"
            and msg.device_info.axis_id == axis_id
        )

    def gateway_info_request(gateway_id: int) -> ffb_protocol.Message:
        m = ffb_protocol.Message()
        m.device_info_request.gateway_id = gateway_id
        return m

    def gateway_matches(msg: ffb_protocol.Message, gateway_id: int) -> bool:
        return (
            msg.device_info.WhichOneof("source") == "gateway_id"
            and msg.device_info.gateway_id == gateway_id
        )

    overall_ok = True

    if axes_targeted:
        if axis_ids:
            ok = await update_target_group(
                protocol,
                axis_ids,
                "axis",
                lambda tid: ffb_protocol.AxisID.Name(tid),
                expected_version,
                lambda tid: build_axis_start_ota_message(args, info_url, tid),
                axis_info_request,
                axis_matches,
                args.retry,
                args.retry_timeout,
                args.poll_interval,
                args.verbose,
            )
            if not ok:
                overall_ok = False
                print("OTA: axis update did not reach expected version.")
        else:
            print("No axes responded during assessment; skipping axis OTA.")

    if gateway_targeted:
        gateway_ids_to_update = present_gateways or GATEWAY_IDS
        ok = await update_target_group(
            protocol,
            gateway_ids_to_update,
            "gateway",
            lambda tid: ffb_protocol.GatewayID.Name(tid),
            expected_version,
            lambda tid: build_gateway_start_ota_message(args, info_url),
            gateway_info_request,
            gateway_matches,
            args.retry,
            args.retry_timeout,
            args.poll_interval,
            args.verbose,
        )
        if not ok:
            overall_ok = False
            print("OTA: gateway update did not reach expected version.")

    transport.close()
    if server:
        server.shutdown()
        server.server_close()
        if thread:
            thread.join(timeout=1.0)
    return 0 if overall_ok else 2


def main() -> int:
    from dotenv import load_dotenv
    load_dotenv()  # take environment variables from .env.
    import os

    parser = argparse.ArgumentParser(description="Trigger OTA updates over USB (gateway forwards to axes).")
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0 or COM3")
    parser.add_argument("--ssid", default=None, help="WiFi SSID (defaults to .env WIFI_SSID)")
    parser.add_argument("--password", default=None, help="WiFi password (defaults to .env WIFI_PASS)")
    parser.add_argument("--url", help="OTA info_json_url")
    parser.add_argument("--firmware", help="Firmware container (.ffbota) to host over HTTP")
    parser.add_argument("--bind", default="0.0.0.0", help="HTTP server bind address")
    parser.add_argument("--host", default="", help="Host/IP to embed in OTA URLs (auto-detect if empty)")
    parser.add_argument("--serve-port", type=int, default=8000, help="HTTP server port")
    parser.add_argument("--allow-downgrades", action="store_true", help="Allow firmware downgrades")
    parser.add_argument("--target", choices=("all", "axes", "gateway"), default="all", help="OTA target scope (default: all). Axes are always updated first, then gateways.")
    parser.add_argument("--axis", type=int, help="Target a specific axis (1..8)")
    parser.add_argument("--retry", type=int, default=1, help="Retry attempts after the initial send (default: 1, matching SimHub)")
    parser.add_argument("--retry-timeout", type=float, default=30.0, help="Seconds to poll DeviceInfo per attempt before retrying (default: 30, matching SimHub)")
    parser.add_argument("--poll-interval", type=float, default=1.0, help="DeviceInfo poll cadence in seconds (default: 1.0, matching SimHub)")
    parser.add_argument("--device-info-timeout", type=float, default=8.0, help="Seconds to wait for the startup discovery DeviceInfo replies")
    parser.add_argument("--baud", type=int, default=3000000, help="Serial baud rate")
    parser.add_argument("--verbose", action="store_true", help="Print non-log messages")
    args = parser.parse_args()

    if not args.url and not args.firmware:
        parser.error("either --url or --firmware is required")

    if args.ssid is None:
        env_ssid = os.getenv("WIFI_SSID", "")
        if env_ssid:
            args.ssid = env_ssid
        else:
            parser.error("SSID is required (pass --ssid or set WIFI_SSID in your environment or .env)")
    if args.password is None:
        env_pass = os.getenv("WIFI_PASS", "")
        if env_pass:
            args.password = env_pass
        else:
            parser.error("password is required (pass --password or set WIFI_PASS in your environment or .env)")
    if args.axis is not None and (args.axis < 1 or args.axis > MAX_AXES):
        parser.error(f"--axis must be within 1..{MAX_AXES}")
    if args.target == "gateway" and args.axis is not None:
        parser.error("--axis cannot be used with --target gateway")

    print("Effective arguments:")
    for (name, value) in args._get_kwargs():
        print(f"  {name} = {value}")

    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 1


if __name__ == "__main__":
    sys.exit(main())
