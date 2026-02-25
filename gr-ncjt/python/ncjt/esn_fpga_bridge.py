#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 Wireless @ Virginia Tech.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


# ESN/FPGA UDP Bridge (general block)
# - Vector in (VLEN=frame_len), vector out (VLEN=out_len)
# - Sends header+weights to cfg port (data_port+1000) and frame to data port
# - Decodes reply as float32 or Q24

import socket
import struct
import io
import math
import numpy as np
from gnuradio import gr

_TENSOR_SHAPE = (2, 2, 2, 6)
_TENSOR_SIZE = int(np.prod(_TENSOR_SHAPE))

_MAGIC = b"NPYB"
_VER = 1
_TYPE_INIT = 1
_TYPE_DATA = 2
_INIT_FMT = "!4sBBH I I I I I I I"
_DATA_FMT = "!4sBBH I I I H I H"

_RX_HEADER_ID = b"BLK64___"
_RX_HEADER_FMT = struct.Struct("!8sI")
_RX_HEADER_LEN = _RX_HEADER_FMT.size

_BLOCK_TENSORS = 10
_TX_CHUNK_SIZE = 1200



def _int16_to_bytes(arr: np.ndarray) -> bytes:
    assert arr.dtype == np.int16
    return arr.tobytes(order="C")

def _bytes_to_float32(buf: bytes) -> np.ndarray:
    n = len(buf) // 4
    return np.frombuffer(buf[:4*n], dtype="<f4")

def _bytes_to_q24(buf: bytes) -> np.ndarray:
    n = len(buf) // 4
    raw = np.frombuffer(buf[:4*n], dtype="<i4")
    return (raw.astype(np.float32) / (1 << 24)).astype(np.float32)

def _bytes_to_int16(buf: bytes) -> np.ndarray:
    n = len(buf) // 2
    return np.frombuffer(buf[:2*n], dtype="<i2")
    
def _block_to_npy_bytes(block: np.ndarray) -> bytes:
    buf = io.BytesIO()
    np.save(buf, block, allow_pickle=False)
    return buf.getvalue()

class esn_fpga_bridge(gr.basic_block):
    """
    ESN/FPGA UDP Bridge (general block)

    Parameters
    ----------
    fpga_ip : str
        FPGA IPv4 address (e.g., "192.168.20.20")
    data_port : int
        UDP data port (config uses data_port+1000)
    frame_len : int
        Input vector length (VLEN in)
    out_len : int
        Output vector length (VLEN out)
    q15_scale : float
        Scale factor for input float->int16 (default 32767.0)
    output_format : str
        "float32" or "q24" (how to decode reply)
    socket_timeout_ms : int
        UDP recv timeout in milliseconds
    default_weights : list|tuple|np.ndarray|None
        Optional initial weights (int16 expected; floats will be rounded/clipped)
    update_weights_each_packet : bool
        If True, send (header+weights) to cfg port for every frame. If False, send once at start or when updated.
    bind_to_data_port : bool
        If True, bind RX socket to data_port; else bind to ephemeral.
    read_from_file : bool
        If True, ignore stream input and continuously read payloads from file for UDP transmit.
    npz_path : str
        .npz path containing real input data to be preloaded into a tensor buffer.
    npz_key : str
        Key within the .npz file (defaults to first key when omitted).
    """

    def __init__(self,
		 length_tag_key='packet_len',
                 fpga_ip="192.168.20.20",
                 data_port=8001,
                 frame_len=256,
                 out_len=128,
                 q15_scale=32767.0,
                 output_format="float32",
                 socket_timeout_ms=250,
                 default_weights=None,
                 update_weights_each_packet=True,
                 bind_to_data_port=True,
                 read_from_file=True,
                 npz_path="",
                 npz_key=""):

        if isinstance(read_from_file, str):
            self.read_from_file = read_from_file.strip().lower() == "true"
        else:
            self.read_from_file = bool(read_from_file)

        # Keep one stream input port regardless of mode so a GRC flowgraph can
        # remain connected even when read_from_file=True.
        #
        # In file mode we simply ignore input samples in general_work.
        in_sig = [(np.float32, int(frame_len))]
        
        gr.basic_block.__init__(
            self,
            name="esn_fpga_bridge",
            in_sig=in_sig,
            out_sig=[(np.float32, int(out_len))],
        )

        # Params
        self.fpga_ip = str(fpga_ip)
        self.data_port = int(data_port)
        self.cfg_port = self.data_port + 1000
        self.frame_len = int(frame_len)
        self.out_len = int(out_len)
        self.q15_scale = float(q15_scale)
        self.output_format = str(output_format).lower()
        self.timeout = max(1, int(socket_timeout_ms)) / 1000.0
        self.update_each = bool(update_weights_each_packet)
        self.bind_to_data_port = bool(bind_to_data_port)
        self.npz_path = str(npz_path or "")
        self.npz_key = str(npz_key or "")

        # Protocol header (matches your fpgaESN.py style)
        self.header = np.array([32670, 0, 64, 0], dtype=np.int16)

        # UDP socket
        self.addr_data = (self.fpga_ip, self.data_port)
        self.addr_cfg = (self.fpga_ip, self.cfg_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        if self.bind_to_data_port:
            try:
                self.sock.bind(("", self.data_port))
            except OSError:
                # Fall back to ephemeral if in use
                self.sock.bind(("", 0))
        else:
            self.sock.bind(("", self.data_port+1))

        # Weights buffer (int16)
        self.weights = None
        if default_weights is not None:
            w = np.array(default_weights)
            if w.dtype != np.int16:
                w = np.clip(np.rint(w), -32768, 32767).astype(np.int16)
            self.weights = w.reshape(-1).astype(np.int16)

        # One-time weight push if not doing per-frame
        if self.weights is not None and not self.update_each:
            self._send_weights_once()

        # Optional message port for runtime weight updates
        try:
            import pmt
            self.message_port_register_in(pmt.intern("weights"))
            self.set_msg_handler(pmt.intern("weights"), self._on_weights_msg)
        except Exception:
            # PMT might not be available in some minimal environments
            pass

        self._warned_timeout = False
        self.tensor_buffer = None
        self.tensor_cursor = 0
        self.tensors_per_packet = _BLOCK_TENSORS
        self._session_id = 1
        self._init_sent = False


        if self.npz_path:
            if self.read_from_file and self.tensors_per_packet < 1:
                raise ValueError(f"frame_len={self.frame_len} must be at least {_TENSOR_SIZE} when npz_path is set")
            self.tensor_buffer = self._load_tensor_buffer(self.npz_path, self.npz_key)

            if self.read_from_file and self.tensor_buffer is None:
                raise ValueError("read_from_file=True requires npz_path to be set")

    # ---------- lifecycle ----------
    def stop(self):
        try:
            self.sock.close()
        except Exception:
            pass
        return super().stop()

    # ---------- forecast ----------
    def forecast(self, noutput_items, ninputs):
        # 1 input frame produces 1 output frame
        return [min(1, noutput_items)] * ninputs

    # ---------- helpers ----------
    def _send_weights_once(self):
        if self.weights is not None and self.weights.size > 0:
            payload = np.concatenate([self.header, self.weights.reshape(-1)]).astype(np.int16)
        else:
            payload = self.header
        try:
            self.sock.sendto(_int16_to_bytes(payload), self.addr_cfg)
        except Exception:
            # Non-fatal; will try again next time if update_each is True
            pass

    def _quantize_to_int16(self, x_f32: np.ndarray) -> np.ndarray:
        y = np.clip(x_f32, -1.0, 1.0) * self.q15_scale
        return np.clip(np.rint(y), -32768, 32767).astype(np.int16)

    def _decode_reply(self, data: bytes) -> np.ndarray:
        if self.output_format == "q24":
            y = _bytes_to_q24(data)
        elif self.output_format == "int16":
            y = _bytes_to_int16(data).astype(np.float32) # de-quantize below
            y = y / self.q15_scale
        else:
            y = _bytes_to_float32(data)
        if y.size < self.out_len:
            z = np.zeros(self.out_len, dtype=np.float32)
            z[:y.size] = y.astype(np.float32)
            return z
        elif y.size > self.out_len:
            return y[:self.out_len].astype(np.float32)
        else:
            return y.astype(np.float32)
        
    def _load_tensor_buffer(self, npz_path: str, npz_key: str) -> np.ndarray:
        npz = np.load(npz_path)
        if isinstance(npz, np.lib.npyio.NpzFile):
            keys = npz.files
            if not keys:
                raise ValueError(f"No arrays found in NPZ file: {npz_path}")
            if npz_key:
                if npz_key not in keys:
                    raise KeyError(f"npz_key '{npz_key}' not found in {npz_path}; available keys: {keys}")
                arr = npz[npz_key]
            else:
                arr = npz[keys[0]]
        else:
            arr = npz

        if np.iscomplexobj(arr):
            raise TypeError("Loaded NPZ array must be real-valued (not complex)")

        flat = np.asarray(arr, dtype=np.float64).reshape(-1)
        ntensors = flat.size // _TENSOR_SIZE
        if ntensors < 1:
            raise ValueError(
                f"Loaded real data has {flat.size} elements; need at least {_TENSOR_SIZE} for one 2x2x2x6 tensor"
            )

        if ntensors < self.tensors_per_packet:
            raise ValueError(
                f"Need at least {self.tensors_per_packet} tensors in file, found {ntensors}"
            )

        used_tensors = (ntensors // self.tensors_per_packet) * self.tensors_per_packet
        used = used_tensors * _TENSOR_SIZE

        if used != flat.size:
            print(
                f"[esn_fpga_bridge] Dropping {flat.size - used} samples so tensor count is a multiple of {self.tensors_per_packet}"
            )

        return flat[:used].reshape(used_tensors, *_TENSOR_SHAPE)

    def _next_tensor_block(self):
        start = self.tensor_cursor
        end = start + self.tensors_per_packet
        block = self.tensor_buffer[start:end]
        if len(block) < self.tensors_per_packet:
            remain = self.tensors_per_packet - len(block)
            block = np.concatenate([block, self.tensor_buffer[:remain]], axis=0)
            self.tensor_cursor = remain
        else:
            self.tensor_cursor = end % len(self.tensor_buffer)
        return start, block.astype(np.float64)

    def _send_init_packet(self):
        if self._init_sent or self.tensor_buffer is None:
            return

        total_tensors = int(len(self.tensor_buffer))
        d1, d2, d3, d4 = _TENSOR_SHAPE
        init_pkt = struct.pack(
            _INIT_FMT,
            _MAGIC,
            _VER,
            _TYPE_INIT,
            0,
            self._session_id,
            total_tensors,
            self.tensors_per_packet,
            d1,
            d2,
            d3,
            d4,
        )

        for _ in range(3):
            self.sock.sendto(init_pkt, self.addr_data)
        self._init_sent = True

    def _send_tensor_block(self, start: int, block: np.ndarray):
        npy_bytes = _block_to_npy_bytes(block)
        npy_len = len(npy_bytes)
        n_chunks = math.ceil(npy_len / _TX_CHUNK_SIZE)

        for seq in range(n_chunks):
            off = seq * _TX_CHUNK_SIZE
            chunk = npy_bytes[off:off + _TX_CHUNK_SIZE]
            pkt = struct.pack(
                _DATA_FMT,
                _MAGIC,
                _VER,
                _TYPE_DATA,
                0,
                self._session_id,
                start,
                npy_len,
                _TX_CHUNK_SIZE,
                seq,
                len(chunk),
            ) + chunk
            self.sock.sendto(pkt, self.addr_data)

    def _recv_file_header(self):
        while True:
            data, _ = self.sock.recvfrom(65535)
            if len(data) < _RX_HEADER_LEN:
                continue
            file_id, file_size = _RX_HEADER_FMT.unpack_from(data, 0)
            if file_id != _RX_HEADER_ID:
                continue
            return file_size

    def _recv_file_payload(self, nbytes: int) -> bytes:
        chunks = []
        got = 0
        while got < nbytes:
            data, _ = self.sock.recvfrom(65535)
            chunks.append(data)
            got += len(data)
        return b"".join(chunks)[:nbytes]

    def _decode_file_reply(self) -> np.ndarray:
        file_size = self._recv_file_header()
        payload = self._recv_file_payload(file_size)
        arr = np.frombuffer(payload, dtype=np.float64)
        y = arr.astype(np.float32)
        if y.size < self.out_len:
            z = np.zeros(self.out_len, dtype=np.float32)
            z[:y.size] = y
            return z
        return y[:self.out_len]

    # ---------- message port: weights ----------
    def _on_weights_msg(self, pdu):
        try:
            import pmt
        except Exception:
            return
        if pdu is None:
            return

        # Accept PDU pair with u8 payload (raw int16 little-endian)
        if pmt.is_pair(pdu):
            vec = pmt.cdr(pdu)
            if pmt.is_u8vector(vec):
                ba = bytes(bytearray(pmt.u8vector_elements(vec)))
                w = np.frombuffer(ba, dtype="<i2")
                self.weights = w.astype(np.int16).copy()
                if not self.update_each:
                    self._send_weights_once()
                return

        # Or typed vectors
        if pmt.is_s16vector(pdu):
            w = np.array(pmt.s16vector_elements(pdu), dtype=np.int16)
            self.weights = w.reshape(-1)
            if not self.update_each:
                self._send_weights_once()
            return
        if pmt.is_f32vector(pdu):
            w = np.array(pmt.f32vector_elements(pdu), dtype=np.float32)
            self.weights = np.clip(np.rint(w), -32768, 32767).astype(np.int16).reshape(-1)
            if not self.update_each:
                self._send_weights_once()
            return

    # ---------- main ----------
    def general_work(self, input_items, output_items):
        """
        output_items[0]: shape (n_out_frames, out_len)
        input_items[0] has shape (n_in_frames, frame_len).
        If read_from_file=True, stream input is ignored and frames are sourced
        from file.
        """
        yout = output_items[0]

        produced = 0
        if self.read_from_file:
            nframes = len(yout)
            xin = None
        else:
            xin = input_items[0]
            nframes = min(len(xin), len(yout))

        for k in range(nframes):
            frame = None if self.read_from_file else xin[k]

            # Update weights this frame if requested
            if self.update_each:
                self._send_weights_once()

            # Quantize and send
            try:
                if not self.read_from_file:
                    q = self._quantize_to_int16(frame.astype(np.float32))
                    self.sock.sendto(_int16_to_bytes(q), self.addr_data)
                    data, _ = self.sock.recvfrom(65535)
                    y = self._decode_reply(data)
                else:
                    self._send_init_packet()
                    start, block = self._next_tensor_block()
                    self._send_tensor_block(start, block)
                    y = self._decode_file_reply()

                self._warned_timeout = False
            except socket.timeout:
                if not self._warned_timeout:
                    print("[esn_fpga_bridge] UDP recv timeout; emitting zeros")
                    self._warned_timeout = True
                y = np.zeros(self.out_len, dtype=np.float32)
            except Exception as e:
                print(f"[esn_fpga_bridge] ERROR: {e}")
                y = np.zeros(self.out_len, dtype=np.float32)

            yout[k, :] = y
            produced += 1

        # Consume input frames and report produced outputs
        if not self.read_from_file:
            self.consume(0, produced)
        return produced
