#!/usr/bin/env python3
"""
Interactive BLE client for the MinRC1 test-stepper firmware (Nordic UART).

Install deps once (use a venv on macOS/Homebrew Python):
  python3 -m venv .venv && source .venv/bin/activate
  pip install -r requirements.txt

Usage:
  python3 stepper_ble_control.py              # scan by name MinRC1-Stepper
  python3 stepper_ble_control.py --scan       # list nearby BLE devices
  python3 stepper_ble_control.py -a AA:BB:... # connect by address

Commands sent to the board (same as USB serial):
  d<0-100>   duty percent
  f<kHz>     PWM frequency in kHz
  p<ms>      step period in ms

Examples:
  d80
  f20
  p10
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from typing import Optional

# Nordic UART (matches firmware)
UART_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def _notify_handler(_sender: object, data: bytearray) -> None:
    print(data.decode("utf-8", errors="replace"), end="", flush=True)


async def _run(address: Optional[str], name: str) -> None:
    from bleak import BleakClient, BleakScanner

    if address:
        device = await BleakScanner.find_device_by_address(address, timeout=15.0)
    else:
        device = await BleakScanner.find_device_by_name(name, timeout=20.0)

    if device is None:
        print("Device not found.", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {device.name or '?'} [{device.address}] ...")

    async with BleakClient(device) as client:
        await client.start_notify(UART_TX, _notify_handler)
        print(
            "Connected. Commands: d<duty%%>  f<freq_khz>  p<period_ms>  (e.g. d80, f20, p10)\n"
            "Type quit to exit.\n"
        )
        loop = asyncio.get_event_loop()
        while True:
            line = await loop.run_in_executor(None, lambda: input("> "))
            line = line.strip()
            if line.lower() in ("q", "quit", "exit"):
                break
            if not line:
                continue
            payload = (line + "\n").encode("utf-8")
            await client.write_gatt_char(UART_RX, payload, response=False)


async def _scan() -> None:
    from bleak import BleakScanner

    print("Scanning 10s ...")
    devices = await BleakScanner.discover(timeout=10.0)
    for d in devices:
        print(f"  {d.address}  {d.name or ''}")


def main() -> None:
    parser = argparse.ArgumentParser(description="BLE control for MinRC1 stepper test")
    parser.add_argument(
        "-a",
        "--address",
        help="BLE address (skip name scan)",
    )
    parser.add_argument(
        "-n",
        "--name",
        default="MinRC1-Stepper",
        help="Advertised name to find (default: %(default)s)",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="List devices and exit",
    )
    args = parser.parse_args()

    if args.scan:
        asyncio.run(_scan())
        return

    asyncio.run(_run(args.address, args.name))


if __name__ == "__main__":
    main()
