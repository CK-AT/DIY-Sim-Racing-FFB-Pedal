#!/usr/bin/env python3
import argparse
import asyncio
import hashlib
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

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


def make_handler(json_bytes: bytes, firmware_path: Path, verbose: bool):
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
                try:
                    size = firmware_path.stat().st_size
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Length", str(size))
                    self.end_headers()
                    with firmware_path.open("rb") as fh:
                        while True:
                            chunk = fh.read(16384)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                except OSError:
                    self.send_error(500, "failed to read firmware")
                return
            self.send_error(404, "not found")

        def log_message(self, fmt, *args):
            if verbose:
                super().log_message(fmt, *args)

    return OtaHandler


def compute_md5_hex(path: Path) -> str:
    md5 = hashlib.md5()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            md5.update(chunk)
    return md5.hexdigest()


def start_ota_server(firmware_path: Path, board: str, version: str, bind: str, host: str, port: int, verbose: bool):
    host_ip = resolve_host_ip(host, bind)
    firmware_url = f"http://{host_ip}:{port}/firmware.bin"
    md5_hex = compute_md5_hex(firmware_path)
    info_payload = {"Configurations": [{"Board": board, "Version": version, "URL": firmware_url, "MD5": md5_hex}]}
    json_bytes = json.dumps(info_payload).encode("ascii")
    handler = make_handler(json_bytes, firmware_path, verbose)
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


async def monitor_logs(protocol: OutputProtocol, seconds: float, verbose: bool) -> None:
    if seconds <= 0:
        return
    deadline = time.time() + seconds
    while time.time() < deadline:
        timeout = max(0.1, deadline - time.time())
        try:
            msg = await asyncio.wait_for(protocol.msg_queue.get(), timeout)
        except asyncio.TimeoutError:
            continue
        log_message(msg, verbose)


def build_start_ota_message(args, info_url: str, target: int, target_axis_id: int | None = None) -> ffb_protocol.Message:
    msg = ffb_protocol.Message()
    msg.start_ota_update.wifi_info.ssid = args.ssid
    msg.start_ota_update.wifi_info.password = args.password
    msg.start_ota_update.info_json_url = info_url
    msg.start_ota_update.allow_downgrades = args.allow_downgrades
    msg.start_ota_update.target = target
    if target_axis_id is not None:
        msg.start_ota_update.target_axis_id = target_axis_id
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


def find_failed_axes(device_info: dict[int, ffb_protocol.DeviceInfo], axis_ids: list[int], expected_version: str) -> list[int]:
    failed: list[int] = []
    for axis_id in axis_ids:
        info = device_info.get(axis_id)
        if info is None:
            failed.append(axis_id)
            continue
        if info.fw_version.strip() != expected_version:
            failed.append(axis_id)
    return failed


async def main_async(args) -> int:
    server = None
    thread = None
    info_url = args.url
    if args.firmware:
        firmware_path = Path(args.firmware).expanduser()
        if not firmware_path.exists():
            raise FileNotFoundError(f"firmware not found: {firmware_path}")
        server, thread, info_url = start_ota_server(
            firmware_path=firmware_path,
            board=args.board,
            version=args.version,
            bind=args.bind,
            host=args.host,
            port=args.serve_port,
            verbose=args.verbose,
        )
        print(f"Serving OTA at {info_url}")

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

    if args.gateway_last:
        if axis_ids:
            print("Starting OTA for axes...")
            can_broadcast_axes = args.axis is None and len(axis_ids) == len(requested_axis_ids)
            if can_broadcast_axes:
                protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["axes"]))
            else:
                for axis_id in axis_ids:
                    protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["axes"], target_axis_id=axis_id))
                    await asyncio.sleep(0.02)
            await monitor_logs(protocol, args.monitor, args.verbose)
            for attempt in range(args.retry):
                device_info, _ = await request_device_info(protocol, axis_ids, [], args.device_info_timeout, args.verbose)
                failed_axes = find_failed_axes(device_info, axis_ids, args.expected_version)
                if not failed_axes:
                    break
                print(f"Retrying OTA for {len(failed_axes)} axis(es)...")
                for axis_id in failed_axes:
                    axis_name = ffb_protocol.AxisID.Name(axis_id)
                    print(f" -> {axis_name}")
                    protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["axes"], target_axis_id=axis_id))
                    await monitor_logs(protocol, args.retry_monitor, args.verbose)
        else:
            print("No axes responded during assessment; skipping axis OTA.")
        print("Starting OTA for gateway...")
        protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["gateway"]))
        await monitor_logs(protocol, args.monitor, args.verbose)
    else:
        target = OTA_TARGETS["axes"] if (args.axis and args.target == "all") else OTA_TARGETS[args.target]
        if target == OTA_TARGETS["gateway"] and args.axis is not None:
            raise ValueError("--axis cannot be used with --target gateway")
        axes_targeted = target in (OTA_TARGETS["all"], OTA_TARGETS["axes"])
        gateway_targeted = target in (OTA_TARGETS["all"], OTA_TARGETS["gateway"])
        can_use_combined_all = target == OTA_TARGETS["all"] and args.axis is None and len(axis_ids) == len(requested_axis_ids)
        if axes_targeted and can_use_combined_all:
            protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["all"]))
            await monitor_logs(protocol, args.monitor, args.verbose)
        else:
            if axes_targeted:
                if axis_ids:
                    can_broadcast_axes = args.axis is None and len(axis_ids) == len(requested_axis_ids)
                    if can_broadcast_axes:
                        protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["axes"]))
                    else:
                        for axis_id in axis_ids:
                            protocol.send_message(
                                build_start_ota_message(args, info_url, OTA_TARGETS["axes"], target_axis_id=axis_id)
                            )
                            await asyncio.sleep(0.02)
                    await monitor_logs(protocol, args.monitor, args.verbose)
                else:
                    print("No axes responded during assessment; skipping axis OTA.")
            if gateway_targeted:
                protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["gateway"]))
                await monitor_logs(protocol, args.monitor, args.verbose)
        if args.retry and axes_targeted and axis_ids:
            for attempt in range(args.retry):
                device_info, _ = await request_device_info(protocol, axis_ids, [], args.device_info_timeout, args.verbose)
                failed_axes = find_failed_axes(device_info, axis_ids, args.expected_version)
                if not failed_axes:
                    break
                print(f"Retrying OTA for {len(failed_axes)} axis(es)...")
                for axis_id in failed_axes:
                    axis_name = ffb_protocol.AxisID.Name(axis_id)
                    print(f" -> {axis_name}")
                    protocol.send_message(build_start_ota_message(args, info_url, OTA_TARGETS["axes"], target_axis_id=axis_id))
                    await monitor_logs(protocol, args.retry_monitor, args.verbose)
    transport.close()
    if server:
        server.shutdown()
        server.server_close()
        if thread:
            thread.join(timeout=1.0)
    return 0


def main() -> int:
    from dotenv import load_dotenv
    load_dotenv()  # take environment variables from .env.
    import os

    parser = argparse.ArgumentParser(description="Trigger OTA updates over USB (gateway forwards to axes).")
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0 or COM3")
    parser.add_argument("--ssid", default=None, help="WiFi SSID (defaults to .env WIFI_SSID)")
    parser.add_argument("--password", default=None, help="WiFi password (defaults to .env WIFI_PASS)")
    parser.add_argument("--url", help="OTA info_json_url")
    parser.add_argument("--firmware", help="Firmware binary to host over HTTP")
    parser.add_argument("--board", default="CK-AT_A6_V1.0", help="Board name for OTA JSON")
    parser.add_argument("--version", default="9999.0.0", help="Version string for OTA JSON")
    parser.add_argument("--bind", default="0.0.0.0", help="HTTP server bind address")
    parser.add_argument("--host", default="", help="Host/IP to embed in OTA URLs (auto-detect if empty)")
    parser.add_argument("--serve-port", type=int, default=8000, help="HTTP server port")
    parser.add_argument("--allow-downgrades", action="store_true", help="Allow firmware downgrades")
    parser.add_argument("--target", choices=("all", "axes", "gateway"), default="all", help="OTA target (default: all)")
    parser.add_argument("--axis", type=int, help="Target a specific axis (1..8)")
    parser.add_argument("--gateway-last", action="store_true", help="Update axes first, then the gateway")
    parser.add_argument("--retry", type=int, default=1, help="Retry failed axis updates this many times")
    parser.add_argument("--expected-version", default="", help="Expected fw version for device info checks (defaults to --version)")
    parser.add_argument("--device-info-timeout", type=float, default=8.0, help="Seconds to wait for device info replies")
    parser.add_argument("--retry-monitor", type=float, default=30.0, help="Seconds to watch logs per retry")
    parser.add_argument("--baud", type=int, default=3000000, help="Serial baud rate")
    parser.add_argument("--monitor", type=float, default=60.0, help="Log messages for this many seconds")
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
    if args.expected_version:
        args.expected_version = args.expected_version.strip()
    else:
        args.expected_version = args.version
    if args.axis is not None and (args.axis < 1 or args.axis > MAX_AXES):
        parser.error(f"--axis must be within 1..{MAX_AXES}")

    print("Effective arguments:")
    for (name, value) in args._get_kwargs():
        print(f"  {name} = {value}")

    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 1


if __name__ == "__main__":
    sys.exit(main())
