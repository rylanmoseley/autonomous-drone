#!/usr/bin/env python3
"""
/opt/drone/scripts/mavlink_proxy.py
Lightweight MAVLink Serial <-> UDP Proxy & Router for Autonomous Drone SBC.

Bridges:
- Serial UART (/dev/flightcontroller @ 921600 baud)
- Autopilot Daemon UDP (RX: 127.0.0.1:14550, TX: 127.0.0.1:14551)
- Wi-Fi Ground Control Station (GCS) UDP Broadcast (192.168.4.255:14550)
- Interactive GCS UDP Inbound (0.0.0.0:14555)
"""

import sys
import os
import time
import socket
import select
import argparse

SERIAL_CANDIDATES = [
    "/dev/flightcontroller",
    "/dev/ttyS4",
    "/dev/ttyAMA0",
    "/dev/ttyACM0",
    "/dev/ttyUSB0"
]

def open_serial(device_path, baudrate=921600):
    try:
        import serial
        ser = serial.Serial(device_path, baudrate=baudrate, timeout=0.01)
        return ser, True
    except ImportError:
        pass

    try:
        import termios
        fd = os.open(device_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        baud_const = getattr(termios, f'B{baudrate}', termios.B921600)
        attrs[4] = baud_const  # ispeed
        attrs[5] = baud_const  # ospeed
        attrs[0] = 0  # iflag raw
        attrs[1] = 0  # oflag raw
        attrs[2] |= (termios.CLOCAL | termios.CREAD | termios.CS8)
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        attrs[3] = 0  # lflag raw
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd, False
    except Exception as e:
        print(f"[MAVLINK-PROXY] Failed to open {device_path} with termios: {e}", file=sys.stderr)
        return None, False

def find_serial_port(requested_path):
    if requested_path and os.path.exists(requested_path):
        return requested_path
    for candidate in SERIAL_CANDIDATES:
        if os.path.exists(candidate):
            return candidate
    return None

def main():
    parser = argparse.ArgumentParser(description="Autonomous Drone MAVLink Router/Proxy")
    parser.add_argument("--serial", default="/dev/flightcontroller", help="Serial port to Flight Controller")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--ap-rx-ip", default="127.0.0.1", help="Autopilot node telemetry destination IP")
    parser.add_argument("--ap-rx-port", type=int, default=14550, help="Autopilot node telemetry destination port")
    parser.add_argument("--ap-tx-port", type=int, default=14551, help="Local UDP port listening for Autopilot commands")
    parser.add_argument("--gcs-broadcast-ip", default="192.168.4.255", help="Wi-Fi AP broadcast IP for GCS telemetry")
    parser.add_argument("--gcs-port", type=int, default=14550, help="GCS telemetry port")
    parser.add_argument("--gcs-in-port", type=int, default=14555, help="GCS incoming command port")
    args = parser.parse_args()

    # Create UDP sockets
    udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Socket listening for Autopilot commands (127.0.0.1:14551)
    sock_ap_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_ap_cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_ap_cmd.bind(("127.0.0.1", args.ap_tx_port))
    sock_ap_cmd.setblocking(False)

    # Socket listening for incoming GCS packets (0.0.0.0:14555)
    sock_gcs_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_gcs_cmd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock_gcs_cmd.bind(("0.0.0.0", args.gcs_in_port))
    except Exception as e:
        print(f"[MAVLINK-PROXY] Notice: GCS inbound port {args.gcs_in_port} bind error: {e}", file=sys.stderr)
    sock_gcs_cmd.setblocking(False)

    print(f"[MAVLINK-PROXY] Starting MAVLink Proxy...")
    print(f"  Autopilot Telemetry -> {args.ap_rx_ip}:{args.ap_rx_port}")
    print(f"  Autopilot Commands  <- 127.0.0.1:{args.ap_tx_port}")
    print(f"  Wi-Fi GCS Broadcast -> {args.gcs_broadcast_ip}:{args.gcs_port}")
    print(f"  Wi-Fi GCS Inbound   <- 0.0.0.0:{args.gcs_in_port}")

    last_gcs_client = None

    while True:
        port_path = find_serial_port(args.serial)
        if not port_path:
            print(f"[MAVLINK-PROXY] Waiting for serial port ({args.serial})...")
            time.sleep(2)
            continue

        print(f"[MAVLINK-PROXY] Connecting to {port_path} at {args.baud} baud...")
        handle, is_pyserial = open_serial(port_path, args.baud)
        if handle is None:
            time.sleep(2)
            continue

        print(f"[MAVLINK-PROXY] Serial port {port_path} connected. Routing active.")
        serial_fd = handle.fileno() if is_pyserial else handle

        try:
            while True:
                rlist = [serial_fd, sock_ap_cmd]
                if sock_gcs_cmd.fileno() != -1:
                    rlist.append(sock_gcs_cmd)

                readable, _, _ = select.select(rlist, [], [], 1.0)

                # 1. FCU Serial -> UDP (Autopilot & GCS)
                if serial_fd in readable:
                    if is_pyserial:
                        data = handle.read(4096)
                    else:
                        try:
                            data = os.read(serial_fd, 4096)
                        except (BlockingIOError, InterruptedError):
                            data = b""

                    if data:
                        # Forward to Autopilot daemon
                        try:
                            udp_out.sendto(data, (args.ap_rx_ip, args.ap_rx_port))
                        except Exception:
                            pass

                        # Broadcast to Wi-Fi GCS
                        try:
                            udp_out.sendto(data, (args.gcs_broadcast_ip, args.gcs_port))
                        except Exception:
                            pass

                        # Unicast to last registered GCS client
                        if last_gcs_client:
                            try:
                                udp_out.sendto(data, last_gcs_client)
                            except Exception:
                                pass
                    elif not is_pyserial and data == b"":
                        # EOF / device unplugged
                        print(f"[MAVLINK-PROXY] Serial port disconnected.")
                        break

                # 2. Autopilot Commands -> FCU Serial
                if sock_ap_cmd in readable:
                    cmd_data, _ = sock_ap_cmd.recvfrom(4096)
                    if cmd_data:
                        if is_pyserial:
                            handle.write(cmd_data)
                            handle.flush()
                        else:
                            os.write(serial_fd, cmd_data)

                # 3. GCS Commands -> FCU Serial
                if sock_gcs_cmd in readable:
                    gcs_data, gcs_addr = sock_gcs_cmd.recvfrom(4096)
                    if gcs_data:
                        last_gcs_client = gcs_addr
                        if is_pyserial:
                            handle.write(gcs_data)
                            handle.flush()
                        else:
                            os.write(serial_fd, gcs_data)

        except Exception as e:
            print(f"[MAVLINK-PROXY] Serial communication loop error: {e}")
        finally:
            if is_pyserial:
                try:
                    handle.close()
                except Exception:
                    pass
            else:
                try:
                    os.close(serial_fd)
                except Exception:
                    pass
            time.sleep(1)

if __name__ == "__main__":
    main()
