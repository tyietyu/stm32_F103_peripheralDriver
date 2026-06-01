#!/usr/bin/env python3
"""Create OTA package for oled_F103.

Package format:
    ota_image_header_t + application binary

The structure layout must stay in sync with oled_F103/BSP/ota_types.h.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import struct
from pathlib import Path


OTA_IMAGE_MAGIC = 0x4F544131
OTA_IMAGE_HEADER_VERSION = 1
OTA_APP_START_ADDR = 0x08004000
OTA_APP_SIZE_BYTES = 44 * 1024
OTA_IMAGE_HEADER_FORMAT = "<IHHII32sIIIII"
OTA_IMAGE_HEADER_SIZE = struct.calcsize(OTA_IMAGE_HEADER_FORMAT)


def parse_u32(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0 or parsed > 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"value out of uint32 range: {value}")
    return parsed


def build_header(app_bin: bytes, app_addr: int, hw_version: int, sw_version: int, flags: int) -> bytes:
    if len(app_bin) == 0:
        raise ValueError("application binary is empty")
    if len(app_bin) > OTA_APP_SIZE_BYTES:
        raise ValueError(f"application binary too large: {len(app_bin)} > {OTA_APP_SIZE_BYTES}")

    image_crc32 = binascii.crc32(app_bin) & 0xFFFFFFFF
    image_sha256 = hashlib.sha256(app_bin).digest()

    header_without_crc = struct.pack(
        OTA_IMAGE_HEADER_FORMAT,
        OTA_IMAGE_MAGIC,
        OTA_IMAGE_HEADER_VERSION,
        OTA_IMAGE_HEADER_SIZE,
        len(app_bin),
        image_crc32,
        image_sha256,
        app_addr,
        hw_version,
        sw_version,
        flags,
        0,
    )
    header_crc32 = binascii.crc32(header_without_crc) & 0xFFFFFFFF

    return struct.pack(
        OTA_IMAGE_HEADER_FORMAT,
        OTA_IMAGE_MAGIC,
        OTA_IMAGE_HEADER_VERSION,
        OTA_IMAGE_HEADER_SIZE,
        len(app_bin),
        image_crc32,
        image_sha256,
        app_addr,
        hw_version,
        sw_version,
        flags,
        header_crc32,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Pack oled_F103 OTA image")
    parser.add_argument("--input", required=True, type=Path, help="APP binary from fromelf --bin")
    parser.add_argument("--output", required=True, type=Path, help="OTA package output path")
    parser.add_argument("--app-addr", default=OTA_APP_START_ADDR, type=parse_u32)
    parser.add_argument("--hw-version", default=0, type=parse_u32)
    parser.add_argument("--sw-version", default=1, type=parse_u32)
    parser.add_argument("--flags", default=0, type=parse_u32)
    args = parser.parse_args()

    app_bin = args.input.read_bytes()
    header = build_header(app_bin, args.app_addr, args.hw_version, args.sw_version, args.flags)
    package = header + app_bin

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(package)

    print(f"input={args.input}")
    print(f"output={args.output}")
    print(f"header_size={len(header)}")
    print(f"image_size={len(app_bin)}")
    print(f"package_size={len(package)}")
    print(f"image_crc32=0x{binascii.crc32(app_bin) & 0xFFFFFFFF:08X}")
    print(f"package_md5={hashlib.md5(package, usedforsecurity=False).hexdigest()}")
    print(f"package_sha256={hashlib.sha256(package).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
