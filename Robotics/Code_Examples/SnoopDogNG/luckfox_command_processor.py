#!/usr/bin/env python3
"""
LuckFox Pico Ultra  ⇆  ESP32
---------------------------------------
Binary protocol on /dev/ttyS3, 115 200 Bd

Frame layout       : 7E  <COBS-encoded TLVs>  CRC-8  7E
CRC-8 polynomial   : 0x07  (CRC-8/ATM)
COBS terminator    : added automatically by encoder
TLVs (Type-Length-Value)
  0x01  TAKE_PHOTO          esp32➜ultra   (no value)
  0x02  START_REC (u32 ms)  esp32➜ultra
  0x03  STOP_REC            esp32➜ultra
  0x04–0x07  Navigation     ultra➜esp32   (STOP,LEFT,RIGHT,REV)
  0x10  ACK (1 byte)        both ways     (0x01=OK, 0x00=FAIL)
  0x11  PHOTO_READY (u32 id) ultra➜esp32
Enable the port once:  sudo luckfox-config  →  Interface ▸ UART ▸ UART7
"""

import serial, struct, time, sys

#Serial port
ser = serial.Serial('/dev/ttyS3', 115_200, timeout=0.1)
print("UART link ready on", ser.port)

START = 0x7E

# CRC-8/ATM helpers
def crc8(data: bytes, poly: int = 0x07, init: int = 0) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c << 1) ^ poly if c & 0x80 else c << 1
            c &= 0xFF
    return c

#  Tiny COBS encode/decode (no external lib)
def cobs_encode(raw: bytes) -> bytes:
    out, idx = bytearray(), 0
    while idx < len(raw):
        nxt = raw.find(b'\x00', idx)
        nxt = len(raw) if nxt == -1 else nxt
        blk_len = nxt - idx
        while blk_len >= 0xFE:
            out += bytes((0xFF,)) + raw[idx:idx+0xFE]
            idx += 0xFE
            blk_len -= 0xFE
        out += bytes((blk_len + 1,)) + raw[idx:idx+blk_len]
        idx += blk_len + 1      # skip zero
    out += b'\x01'
    return bytes(out)

def cobs_decode(enc: bytes) -> bytes:
    out, i = bytearray(), 0
    while i < len(enc):
        code = enc[i]
        if code == 0 or i + code > len(enc) + 1:
            raise ValueError("COBS decode error")
        out += enc[i+1 : i+code]
        i += code
        if code != 0xFF and i < len(enc):
            out += b'\x00'
    return bytes(out)

# TLV helpers 
def build_tlv(t: int, v: bytes = b'') -> bytes:
    return bytes((t, len(v))) + v

def parse_tlvs(payload: bytes):
    i = 0
    while i + 2 <= len(payload):
        t, l = payload[i], payload[i+1]
        v = payload[i+2 : i+2+l]
        i += 2 + l
        yield t, v

# Protocol constants
CMD_TAKE_PHOTO = 0x01
CMD_START_REC  = 0x02
CMD_STOP_REC   = 0x03

NAV_STOP    = 0x04
NAV_LEFT    = 0x05
NAV_RIGHT   = 0x06
NAV_REVERSE = 0x07

ACK_TLV     = 0x10
PHOTO_READY = 0x11

# Application stubs (replace with real camera logic)
def take_photo() -> int:
    print("📸  Taking photo…")
    time.sleep(0.5)
    return int(time.time()) & 0xFFFFFFFF   # fake image-ID

def start_record(ms: int):
    print(f"🎥  Start recording for {ms} ms")

def stop_record():
    print("⏹️   Stop recording")

# Frame TX helper
def send_frame(*tlvs: bytes) -> None:
    payload = b''.join(tlvs)
    enc     = cobs_encode(payload)
    crc     = crc8(enc)
    ser.write(bytes((START,)) + enc + bytes((crc,)) + bytes((START,)))

#  Frame RX generator (yields raw payload)
def read_frames():
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == START:
            if len(buf) >= 2:
                enc, rx_crc = buf[:-1], buf[-1]
                calc_crc = crc8(enc)
                if calc_crc == rx_crc:
                    try:
                        yield cobs_decode(enc)
                    except ValueError as e:
                        print("⚠️  COBS error", e, "RAW", enc.hex())
                else:
                    print(f"⚠️  CRC mismatch calc={calc_crc:02X} rx={rx_crc:02X}",
                          "RAW", enc.hex())
            buf.clear()
        else:
            buf.append(b[0])

# Main loop
for frame in read_frames():
    for t, v in parse_tlvs(frame):
        if t == CMD_TAKE_PHOTO and len(v) == 0:
            img_id = take_photo()
            send_frame(
                build_tlv(ACK_TLV, b'\x01'),
                build_tlv(PHOTO_READY, struct.pack('<I', img_id))
            )

        elif t == CMD_START_REC and len(v) == 4:
            ms = struct.unpack('<I', v)[0]
            start_record(ms)
            send_frame(build_tlv(ACK_TLV, b'\x01'))

        elif t == CMD_STOP_REC:
            stop_record()
            send_frame(build_tlv(ACK_TLV, b'\x01'))

        else:
            print("❓  Unknown TLV", t, v.hex())
