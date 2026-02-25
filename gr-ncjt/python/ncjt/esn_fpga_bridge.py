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
import numpy as np
from gnuradio import gr

_TENSOR_SHAPE = (2, 2, 2, 6)
_TENSOR_SIZE = int(np.prod(_TENSOR_SHAPE))

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
                 npz_path="",
                 npz_key=""):

        gr.basic_block.__init__(
            self,
            name="esn_fpga_bridge",
            in_sig=[(np.float32, int(frame_len))],
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
        self.tensors_per_packet = self.frame_len // _TENSOR_SIZE

        if self.npz_path:
            if self.tensors_per_packet < 1:
                raise ValueError(f"frame_len={self.frame_len} must be at least {_TENSOR_SIZE} when npz_path is set")
            self.tensor_buffer = self._load_tensor_buffer(self.npz_path, self.npz_key)

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

        flat = np.asarray(arr, dtype=np.float32).reshape(-1)
        ntensors = flat.size // _TENSOR_SIZE
        if ntensors < 1:
            raise ValueError(
                f"Loaded real data has {flat.size} elements; need at least {_TENSOR_SIZE} for one 2x2x2x6 tensor"
            )

        used = ntensors * _TENSOR_SIZE
        if used != flat.size:
            print(f"[esn_fpga_bridge] Dropping {flat.size - used} samples that do not fill a complete tensor")

        return flat[:used].reshape(ntensors, *_TENSOR_SHAPE)

    def _next_tensor_payload(self) -> np.ndarray:
        tensor = self.tensor_buffer[self.tensor_cursor]
        self.tensor_cursor = (self.tensor_cursor + 1) % len(self.tensor_buffer)
        return tensor.astype(np.float32).reshape(-1)

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
        input_items[0]: shape (n_in_frames, frame_len)
        output_items[0]: shape (n_out_frames, out_len)
        We process up to min(n_in_frames, n_out_frames) frames per call.
        """
        xin = input_items[0]
        yout = output_items[0]

        nframes = min(len(xin), len(yout))
        produced = 0

        for k in range(nframes):
            frame = xin[k]

            # Update weights this frame if requested
            if self.update_each:
                self._send_weights_once()

            # Quantize and send
            try:
                if self.tensor_buffer is None:
                    q = self._quantize_to_int16(frame.astype(np.float32))
                    self.sock.sendto(_int16_to_bytes(q), self.addr_data)
                    data, _ = self.sock.recvfrom(65535)
                    y = self._decode_reply(data)
                else:
                    replies = []
                    for _ in range(self.tensors_per_packet):
                        payload = self._next_tensor_payload()
                        q = self._quantize_to_int16(payload)
                        self.sock.sendto(_int16_to_bytes(q), self.addr_data)
                        data, _ = self.sock.recvfrom(65535)
                        replies.append(self._decode_reply(data))

                    y = np.mean(np.stack(replies, axis=0), axis=0).astype(np.float32)
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
        self.consume(0, produced)
        return produced
