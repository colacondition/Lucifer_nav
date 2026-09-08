#!/usr/bin/env python3
"""Livox Mid-360(S) discovery via the official broadcast query protocol.

Request (24 B, LE, packed, per Livox SDK2 SdkProtocol::Pack):
  sof u8=0xAA | version u8=0 | length u16=24 | seq_num u32(<0x10000) |
  cmd_id u16=0x0000 | cmd_type u8=0(cmd) | sender_type u8=0(host) |
  rsvd 6B | crc16_h u16 = CRC-16/CCITT-FALSE over bytes[0:18] | crc32_d u32=0
Sent to 255.255.255.255:56000 every 0.2 s.

Response: same header with cmd_type=1(ack), data (24 B) = DetectionData:
  ret_code u8 | dev_type u8 | sn 16B | lidar_ip 4B | cmd_port u16
"""
import socket
import struct
import sys
import time

DETECTION_PORT = 56000

DEV_TYPES = {
    1: "HAP", 2: "HAP", 6: "PA", 8: " Mid-360(legacy type?)",
    9: "Mid-360", 35: "Mid-360s", 40: "Avia2",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_discovery(seq: int) -> bytes:
    pkt = bytearray(24)
    struct.pack_into("<BBH I H BB 6x H I", pkt, 0,
                     0xAA, 0, 24, seq & 0xFFFF, 0x0000, 0, 0, 0, 0)
    struct.pack_into("<H", pkt, 18, crc16_ccitt_false(bytes(pkt[:18])))
    return bytes(pkt)


def parse_response(data: bytes, addr):
    if len(data) < 24 or data[0] != 0xAA:
        return None
    sof, ver, length, seq, cmd_id, cmd_type, sender = struct.unpack_from("<BBH I H BB", data, 0)
    if length > len(data):
        return None
    payload = data[24:length]
    if cmd_id != 0x0000 or cmd_type != 1:  # only discovery acks
        return None
    if len(payload) < 24:
        return None
    ret_code, dev_type = struct.unpack_from("<BB", payload, 0)
    sn = payload[2:18].rstrip(b"\x00").decode(errors="replace")
    ip = ".".join(str(b) for b in payload[18:22])
    cmd_port, = struct.unpack_from("<H", payload, 22)
    return {"ret": ret_code, "dev_type": dev_type, "sn": sn,
            "lidar_ip": ip, "cmd_port": cmd_port, "from": addr[0]}


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", DETECTION_PORT))
    s.settimeout(0.2)

    found = {}
    deadline = time.time() + duration
    seq = 0x1234
    nxt = 0.0
    # 192.168.1.255 = 子网定向广播，按路由表 192.168.1.0/24 走 enp7s0；
    # 受限广播 255.255.255.255 会走默认路由（WiFi），到不了雷达。
    bcast = ("192.168.1.255", DETECTION_PORT)
    while time.time() < deadline:
        if time.time() >= nxt:
            s.sendto(build_discovery(seq), bcast)
            seq += 1
            nxt = time.time() + 0.2
        try:
            data, addr = s.recvfrom(4096)
            r = parse_response(data, addr)
            if r and r["ret"] == 0:
                found[r["lidar_ip"]] = r
        except socket.timeout:
            pass

    if not found:
        print("NO LIVOX LIDAR RESPONDED")
        return
    for ip, r in sorted(found.items()):
        name = DEV_TYPES.get(r["dev_type"], f"unknown({r['dev_type']})")
        print(f"lidar_ip={ip}  model={name}  dev_type={r['dev_type']}  "
              f"sn={r['sn']}  cmd_port={r['cmd_port']}  (reply via {r['from']})")


if __name__ == "__main__":
    main()
