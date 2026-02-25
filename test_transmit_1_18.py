import socket, struct, time, math, io
import numpy as np

DEST_IP = "192.168.1.10"
DEST_PORT = 5001
CHUNK_SIZE = 1200
SESSION_ID = 1

MAGIC = b"NPYB"
VER = 1
TYPE_INIT = 1
TYPE_DATA = 2

INIT_FMT = "!4sBBH I I I I I I I"
DATA_FMT = "!4sBBH I I I H I H"

def block_to_npy_bytes(block: np.ndarray) -> bytes:
    buf = io.BytesIO()
    np.save(buf, block, allow_pickle=False)
    return buf.getvalue()

def main():

    # ---- Load your existing file ----
    full = np.load("H_mag_phase.npy")   # <-- your actual file

    print("Loaded:", full.shape, full.dtype)

    if full.dtype != np.float64:
        raise ValueError("File must contain float64")

    T = full.shape[0]
    B = 10   # or parameterize this

    if T % B != 0:
        raise ValueError("First dimension must be divisible by block size")

    D1, D2, D3, D4 = full.shape[1:]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ---- Send INIT ----
    init_pkt = struct.pack(
        INIT_FMT,
        MAGIC, VER, TYPE_INIT, 0,
        SESSION_ID,
        T, B,
        D1, D2, D3, D4
    )

    for _ in range(3):
        sock.sendto(init_pkt, (DEST_IP, DEST_PORT))
        time.sleep(0.01)

    time.sleep(0.05)

    # ---- Send each block ----
    for start in range(0, T, B):

        block = full[start:start+B, ...]
        npy_bytes = block_to_npy_bytes(block)

        npy_len = len(npy_bytes)
        n_chunks = math.ceil(npy_len / CHUNK_SIZE)

        for seq in range(n_chunks):
            off = seq * CHUNK_SIZE
            chunk = npy_bytes[off:off+CHUNK_SIZE]

            pkt = struct.pack(
                DATA_FMT,
                MAGIC, VER, TYPE_DATA, 0,
                SESSION_ID,
                start,
                npy_len,
                CHUNK_SIZE,
                seq,
                len(chunk)
            ) + chunk

            sock.sendto(pkt, (DEST_IP, DEST_PORT))
            time.sleep(0.001)

        print(f"Sent block starting at {start}")

        time.sleep(0.01)

    sock.close()
    print("All blocks sent.")

if __name__ == "__main__":
    main()
