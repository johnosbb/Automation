import socket
import json

HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 5000       # Must match the ESP32 port

# Define state mappings
STATE_MAP = {
    0: "STATE_IDLE",
    1: "STATE_FORWARD",
    2: "STATE_REVERSE",
    3: "STATE_LEFT",
    4: "STATE_RIGHT",
    5: "STATE_STOP"
}

REASON_MAP = {
    0: "IR_CONTROL",
    1: "CLEAR_PATH",
    2: "OUT_OF_RANGE",
    3: "BLOCKED",
    4: "RECOVERING",
    5: "UNCERTAIN"
}

def decode_state(state_value):
    """Convert numeric state value to human-readable form."""
    try:
        state_value = int(state_value)  # Convert to integer
        return STATE_MAP.get(state_value, f"UNKNOWN ({state_value})")
    except ValueError:
        return f"INVALID ({state_value})"


def decode_reason(reason_value):
    """Convert numeric reason value to human-readable form."""
    try:
        reason_value = int(reason_value)  # Convert to integer
        return REASON_MAP.get(reason_value, f"UNKNOWN ({reason_value})")
    except ValueError:
        return f"INVALID ({reason_value})"


def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow immediate reuse
        server_socket.bind((HOST, PORT))
        server_socket.listen(5)
        print(f"Server listening on {HOST}:{PORT}")

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connected by {client_address}")

            with client_socket, open("car_log.txt", "a") as log_file:
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    
                    log_message = data.decode("utf-8").strip()

                    # Decode JSON message
                    try:
                        log_data = json.loads(log_message)
                        time_ms = log_data["time"]
                        distance_cm = log_data["distance"]
                        state_raw = log_data["state"]
                        reason_raw = log_data["reason"]

                        # Decode state and reason
                        state_decoded = decode_state(state_raw)
                        reason_decoded = decode_reason(reason_raw)

                        # Format the log message
                        formatted_message = f"Time: {time_ms} ms, Distance: {distance_cm} cm, State: {state_decoded}, Reason: {reason_decoded}"
                    except json.JSONDecodeError:
                        formatted_message = f"Malformed JSON received: {log_message}"

                    print(formatted_message)
                    log_file.write(formatted_message + "\n")


if __name__ == "__main__":
    start_server()
