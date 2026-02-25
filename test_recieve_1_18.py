#!/usr/bin/env python3
"""
udp_rx_ps_blocks.py

Receives blocks sent from ZCU104 PS using:
  1) A header packet containing file_header_t:
        char file_id[8];
        uint32_t file_size;   (RECOMMENDED: network byte order / htonl on PS)

  2) Then raw payload bytes streamed in UDP_CHUNK_BYTES-sized UDP datagrams
     (no sequence numbers). We simply accumulate datagrams until file_size bytes.

Payload interpretation:
  - raw float64 bytes (little-endian on Zynq + PC), so np.frombuffer(..., float64) works.

IMPORTANT LIMITATION:
  - Because your send_bytes_chunked() has no seq/ID, this receiver assumes:
      * only one sender
      * sender sends one block at a time
      * packets arrive in order with no loss
    If you need robustness to loss/reorder, add seq numbers to chunks.

Usage:
  python udp_rx_ps_blocks.py
"""

import socket
import struct
import time
import numpy as np

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5002           # must match UDP_SEND_PORT on PS
SOCKET_RCVBUF = 4 * 1024 * 1024

EXPECTED_FILE_ID = b"BLK64___"  # must match "BLK64___" in send_header()
HEADER_LEN = 12

# Choose endianness that matches PS send_header():
# - If PS uses hdr.file_size = htonl(payload_bytes): use "!8sI"
# - If PS sends raw uint32_t without htonl: use "<8sI" on little-endian systems
HEADER_STRUCT = struct.Struct("!8sI")   # recommended
# HEADER_STRUCT = struct.Struct("<8sI") # fallback if you didn't htonl

# Optional reshape: each block is (B,2,2,2,6) => 48 doubles per sample
TRAIL_ELEMS = 48
TRAIL_SHAPE = (2, 2, 2, 6)


def recv_header(sock: socket.socket, timeout_s: float = 10.0):
    sock.settimeout(timeout_s)
    while True:
        data, addr = sock.recvfrom(65535)
        if len(data) < HEADER_LEN:
            continue
        file_id, file_size = HEADER_STRUCT.unpack_from(data, 0)
        return file_id, file_size, addr


def recv_payload(sock: socket.socket, nbytes: int, timeout_s: float = 2.0) -> bytes:
    """Accumulate UDP datagrams until we have nbytes."""
    sock.settimeout(timeout_s)
    chunks = []
    got = 0
    while got < nbytes:
        data, _ = sock.recvfrom(65535)
        chunks.append(data)
        got += len(data)
    blob = b"".join(chunks)
    return blob[:nbytes]


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, SOCKET_RCVBUF)

    print(f"Listening on {LISTEN_IP}:{LISTEN_PORT}")
    print(f"Expecting header: file_id={EXPECTED_FILE_ID!r}, {HEADER_STRUCT.format}, {HEADER_LEN} bytes")

    blocks = 0
    while True:
        try:
            file_id, file_size, addr = recv_header(sock)
        except socket.timeout:
            continue

        if file_id != EXPECTED_FILE_ID:
            print(f"Ignoring header file_id={file_id!r} from {addr}")
            continue

        print(f"\n[{blocks}] Header from {addr}: file_size={file_size} bytes")

        try:
            payload = recv_payload(sock, file_size, timeout_s=2.0)
        except socket.timeout:
            print(f"Timeout while receiving payload (wanted {file_size} bytes)")
            continue

        if len(payload) != file_size:
            print(f"Length mismatch: got {len(payload)} vs {file_size}")
            continue

        # Interpret as float64
        arr = np.frombuffer(payload, dtype=np.float64)
        print(f"Received {arr.size} float64 values. First 5: {arr[:5]}")

        # Optional reshape
        if arr.size % TRAIL_ELEMS == 0:
            B = arr.size // TRAIL_ELEMS
            arr5 = arr.reshape((B,) + TRAIL_SHAPE)
            print(f"Reshaped to {arr5.shape} (B={B}). Example first element: {arr5[0,0,0,0,0]}")
        else:
            print("Not divisible by 48; skipping reshape.")

        blocks += 1


if __name__ == "__main__":
    main()
