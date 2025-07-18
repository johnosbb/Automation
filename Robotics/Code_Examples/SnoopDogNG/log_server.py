#!/usr/bin/env python3
"""
TCP JSON stream logger that copes with concatenated / partial payloads.

Usage:
    python3 stream_logger.py
"""
import json
import socket
from pathlib import Path

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
HOST = "0.0.0.0"    # Listen on all interfaces
PORT = 5000         # Must match the ESP32 port
LOG_FILE = Path("car_log.txt")

# --------------------------------------------------------------------------- #
# Lookup tables
# --------------------------------------------------------------------------- #
STATE_MAP = {
    0: "STATE_IDLE",
    1: "STATE_FORWARD",
    2: "STATE_REVERSE",
    3: "STATE_LEFT",
    4: "STATE_RIGHT",
    5: "STATE_STOP",
}

REASON_MAP = {
    0: "IR_CONTROL",
    1: "CLEAR_PATH",
    2: "OUT_OF_RANGE",
    3: "BLOCKED",
    4: "RECOVERING",
    5: "UNCERTAIN",
    6: "CRASH_DETECT",
}

# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def decode_state(value: int) -> str:
    """Return a human-readable name for *value*."""
    try:
        return STATE_MAP[int(value)]
    except (ValueError, KeyError):
        return f"UNKNOWN ({value})"


def decode_reason(value: int) -> str:
    """Return a human-readable reason string for *value*."""
    try:
        return REASON_MAP[int(value)]
    except (ValueError, KeyError):
        return f"UNKNOWN ({value})"


def handle_json(obj: dict) -> str:
    """Convert a decoded JSON dict into a formatted log line."""
    try:
        time_ms = obj["time"]
        distance_cm = obj["distance"]
        state_decoded = decode_state(obj["state"])
        reason_decoded = decode_reason(obj["reason"])
        rpm = obj["rpm"]
        return (
            f"Time: {time_ms} ms, Distance: {distance_cm} cm, "
            f"State: {state_decoded}, Reason: {reason_decoded}, RPM: {rpm}"
        )
    except KeyError as exc:
        return f"Missing key {exc!s} in JSON: {obj}"


# --------------------------------------------------------------------------- #
# Main server
# --------------------------------------------------------------------------- #
def start_server() -> None:
    LOG_FILE.touch(exist_ok=True)  # Ensure the file exists

    decoder = json.JSONDecoder()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(5)
        print(f"Server listening on {HOST}:{PORT}")

        while True:
            try:
                client, addr = srv.accept()
                print(f"Connected by {addr}")
                with client, LOG_FILE.open("a") as log_file:
                    client.settimeout(10)
                    buffer = ""  # Accumulates partial / multiple JSON objects
                    while True:
                        try:
                            chunk = client.recv(1024)
                            if not chunk:
                                print("Client disconnected.")
                                break

                            buffer += chunk.decode("utf-8")

                            # Extract as many complete JSON objects as possible
                            while buffer:
                                buffer = buffer.lstrip()  # Remove leading whitespace
                                try:
                                    obj, idx = decoder.raw_decode(buffer)
                                except json.JSONDecodeError:
                                    # Not enough data for a complete object yet
                                    break

                                buffer = buffer[idx:]  # Remove parsed text
                                line = handle_json(obj)
                                print(line)
                                log_file.write(line + "\n")

                        except socket.timeout:
                            print("Client connection timed out.")
                            break
                        except ConnectionResetError:
                            print("Connection reset by peer.")
                            break
            except Exception as exc:
                print(f"Error handling connection: {exc}")


# --------------------------------------------------------------------------- #
# Entrypoint
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    start_server()
