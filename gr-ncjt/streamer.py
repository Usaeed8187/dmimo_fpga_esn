import sys
import subprocess
import re
import math
import shutil
import os
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QProcess
from PyQt5.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QSpinBox, QPushButton, QFileDialog, QRadioButton,
    QButtonGroup, QCheckBox, QMessageBox
)
from PyQt5.QtGui import QImage, QPixmap

VIDEO_FILE_PATH = sys.argv[1] if len(sys.argv) > 1 else "sample_video.mp4"
if not os.path.isabs(VIDEO_FILE_PATH):
    VIDEO_FILE_PATH = os.path.abspath(VIDEO_FILE_PATH)

# ============================================================
# 1. Utility to parse supported webcam modes (macOS & Linux)
# ============================================================
def parse_supported_webcam_modes():
    """
    Returns a dict of { 'widthxheight': [fps1, fps2, ...], ... }.
    For macOS, it uses ffmpeg with avfoundation.
    For Linux, it first tries v4l2-ctl; if unavailable, it falls back to the previous ffmpeg method.
    """
    if sys.platform == 'darwin':
        cmd = [
            "ffmpeg",
            "-f", "avfoundation",
            "-i", "0:none",
            "-hide_banner",
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True)
        except Exception as e:
            print("Error running ffmpeg for avfoundation modes:", e)
            return {}
        # Combine stdout and stderr
        all_output = (proc.stdout or "") + "\n" + (proc.stderr or "")
        print("=== FFmpeg raw output for webcam modes ===")
        print(all_output)
        print("=== End of FFmpeg output ===")
        pattern = re.compile(r"(\d+x\d+)@\[(\d+\.\d+)\s+(\d+\.\d+)\]fps")
        modes_dict = {}
        for line in all_output.splitlines():
            line_str = line.strip()
            if not line_str:
                continue
            print("[DEBUG] Processing line:", line_str)
            match = pattern.search(line_str)
            if match:
                resolution = match.group(1)
                fps_min_str = match.group(2)
                fps_max_str = match.group(3)
                try:
                    fps_min = float(fps_min_str)
                    fps_max = float(fps_max_str)
                except ValueError:
                    print("[DEBUG] Could not parse FPS values in line:", line_str)
                    continue

                if abs(fps_min - fps_max) < 0.001:
                    actual_fps = round(fps_min, 2)
                    fps_list = modes_dict.setdefault(resolution, [])
                    if actual_fps not in fps_list:
                        fps_list.append(actual_fps)
                        print(f"[DEBUG] Added mode: {resolution} @ {actual_fps}")
                else:
                    for f in [fps_min, fps_max]:
                        rounded = round(f, 2)
                        fps_list = modes_dict.setdefault(resolution, [])
                        if rounded not in fps_list:
                            fps_list.append(rounded)
                            print(f"[DEBUG] Added mode (non-equal fps): {resolution} @ {rounded}")
            else:
                print("[DEBUG] No match in line:", line_str)
        # Sort each fps list descending
        for res in modes_dict:
            modes_dict[res].sort(reverse=True)
        print("[DEBUG] Final parsed modes dictionary:", modes_dict)
        return modes_dict

    elif sys.platform.startswith('linux'):
        # First, try using v4l2-ctl if available.
        if shutil.which("v4l2-ctl"):
            cmd = ["v4l2-ctl", "--list-formats-ext"]
            try:
                proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
            except Exception as e:
                print("Error running v4l2-ctl:", e)
                proc = None
            if proc is not None:
                output = proc.stdout
                modes_dict = {}
                current_resolution = None
                # Regex to match resolution lines, e.g. "Size: Discrete 640x480"
                resolution_pattern = re.compile(r"Size:\s+Discrete\s+(\d+x\d+)")
                # Regex to match fps lines, e.g. "Interval: Discrete 0.033s (30.000 fps)"
                fps_pattern = re.compile(r"Interval:\s+Discrete\s+[\d\.]+s\s+\(([\d\.]+)\s+fps\)")
                for line in output.splitlines():
                    line = line.strip()
                    res_match = resolution_pattern.search(line)
                    if res_match:
                        current_resolution = res_match.group(1)
                        modes_dict.setdefault(current_resolution, [])
                        continue
                    fps_match = fps_pattern.search(line)
                    if fps_match and current_resolution:
                        try:
                            fps_val = round(float(fps_match.group(1)), 2)
                        except ValueError:
                            continue
                        if fps_val not in modes_dict[current_resolution]:
                            modes_dict[current_resolution].append(fps_val)
                for res in modes_dict:
                    modes_dict[res].sort(reverse=True)
                return modes_dict

        # Fallback: use the ffmpeg approach.
        cmd = [
            "ffmpeg",
            "-f", "v4l2",
            "-list_formats", "all",
            "-i", "/dev/video0"
        ]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True)
        except Exception as e:
            print("Error running ffmpeg for v4l2 modes:", e)
            return {}
        # v4l2 typically prints to stderr; combine stdout and stderr
        all_output = (proc.stdout or "") + "\n" + (proc.stderr or "")
        modes_dict = {}
        # Default fps list (used if detailed fps info is not available)
        default_fps_list = [30.0, 24.0, 20.0, 15.0, 10.0, 7.5, 5.0]
        for line in all_output.splitlines():
            line_str = line.strip()
            if not line_str:
                continue
            # Look for lines containing "Raw" or "Compressed" along with resolutions
            if ("Raw" in line_str or "Compressed" in line_str) and ':' in line_str:
                parts = line_str.split(":")
                if len(parts) < 2:
                    continue
                # e.g., " 640x480 160x90 320x240 ..." -> pick tokens containing an "x"
                last_part = parts[-1]
                tokens = last_part.split()
                for tok in tokens:
                    if "x" in tok and tok.count("x") == 1:
                        modes_dict.setdefault(tok, list(default_fps_list))
        for res in modes_dict:
            modes_dict[res].sort(reverse=True)
        return modes_dict

    else:
        print("Unsupported platform for webcam parsing!")
        return {}


# ============================================================
# 2. Thread: Captures Webcam or File, Encodes, Streams
# ============================================================
class StreamerThread(QThread):
    def __init__(
        self, width=640, height=480, fps=30.0,
        codec='h264', color_mode='color', bitrate=300,
        source_type='webcam', file_path='', audio=True,
        osd_enabled=False, parent=None
    ):
        super().__init__(parent)
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec.lower()
        self.color_mode = color_mode.lower()
        self.bitrate = bitrate
        self.source_type = source_type
        self.file_path = file_path
        self.audio = audio
        self.osd_enabled = osd_enabled
        self._running = True
        self.process = None

    def run(self):
        if self.source_type == 'webcam':
            # macOS
            if sys.platform == 'darwin':
                input_cmd = [
                    'ffmpeg',
                    '-f', 'avfoundation',
                    '-pixel_format', 'uyvy422',
                    '-framerate', str(self.fps),
                    '-video_size', f'{self.width}x{self.height}',
                    '-i', '0:none'
                ]
            # Linux
            elif sys.platform.startswith('linux'):
                input_cmd = [
                    'ffmpeg',
                    '-f', 'v4l2',
                    '-video_size', f'{self.width}x{self.height}',
                    '-framerate', str(self.fps),
                    '-i', '/dev/video0'
                ]
            else:
                print("Unsupported platform for webcam streaming!")
                return

        elif self.source_type == 'file':
            input_cmd = [
                'ffmpeg',
                '-re',
                '-stream_loop', '-1',
                '-i', self.file_path
            ]
        else:
            print("Unknown source type!")
            return
        
        # Common low-latency flags
        input_cmd += [
            '-fflags', '+nobuffer+flush_packets+genpts+igndts',  # disable buffering, flush immediately, generate PTS
            '-avoid_negative_ts', 'make_zero',  # handle timestamp issues
            '-max_delay', '0',  # minimize delay
        ]

        # Prepare filters (scale, color, OSD).
        filters = [f"scale={self.width}:{self.height}"]
        if self.color_mode == 'bw':
            filters.append("format=gray,format=yuv420p")

        if self.osd_enabled:
            osd_text = f"%{{localtime}} - {self.codec.upper()}, {self.bitrate}k, {('Color' if self.color_mode=='color' else 'BW')}"
            drawtext_filter = (
                f"drawtext=fontfile=/path/to/font.ttf:"
                f"text='{osd_text}':x=10:y=10:fontsize=24:"
                f"fontcolor=white:box=1:boxcolor=black@0.5"
            )
            filters.append(drawtext_filter)

        if filters:
            filter_str = ",".join(filters)
            input_cmd += ['-vf', filter_str]

        # Choose encoder
        if self.codec == 'h264':
            input_cmd += [
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-flags', 'low_delay',
                '-x264opts', 'repeat-headers:nal-hrd=cbr'
            ]
        elif self.codec == 'h265':
            x265_params = [
                f'vbv-maxrate={self.bitrate}',
                f'vbv-bufsize={self.bitrate}',  # Reduced buffer size
                'tune=fastdecode',  # Optimize for fast decoding
                # 'no-scenecut',  # Disable scene change detection
                # 'keyint=30',  # Fixed keyframe interval
                # 'min-keyint=30',  # Same as keyint for consistency
                'bframes=0',  # Disable B-frames for lower latency
                # 'weightp=0',  # Disable weighted prediction
                # 'weightb=0',  # Disable weighted biprediction
                # 'ref=1',  # Use only 1 reference frame
                # 'me=dia',  # Use diamond motion estimation (fastest)
                # 'subme=0',  # Disable subpixel motion estimation
                # 'rd=1',  # Minimal rate distortion
                # 'nr-intra=0',  # Disable noise reduction
                # 'nr-inter=0',  # Disable noise reduction
                # 'no-cutree',  # Disable CU tree rate control
                # 'aq-mode=0',  # Disable adaptive quantization
                # 'no-sao',  # Disable Sample Adaptive Offset
                # 'no-deblock',  # Disable deblocking filter (optional, may reduce quality)
                # 'pools=1',  # Use single thread pool
                # 'frame-threads=1',  # Reduce frame threads
                # 'no-wpp',  # Disable Wavefront Parallel Processing
                # 'rect=0',  # Disable rectangular partitions
                # 'amp=0',  # Disable asymmetric motion partitions
            ]
            input_cmd += [
                '-c:v', 'libx265',
                '-preset', 'ultrafast',
                '-x265-params', ':'.join(x265_params)
            ]
        else:
            print("Unsupported codec!")
            return

        # Bitrate, streaming options
        input_cmd += [
            '-b:v', f'{self.bitrate}k',
            # '-minrate', f'{self.bitrate}k',
            '-maxrate', f'{self.bitrate}k',
            '-bufsize', f'{2 * self.bitrate}k'
        ]

        # Disable audio if needed
        if not self.audio:
            input_cmd += ['-an']

        # Bitstream filters for TS
        if self.codec == 'h264':
            input_cmd += ['-bsf:v', 'h264_mp4toannexb']
        elif self.codec == 'h265':
            input_cmd += ['-bsf:v', 'hevc_mp4toannexb']

        input_cmd += [
            '-mpegts_flags', '+resend_headers',
            '-muxdelay', '0',
            '-omit_video_pes_length', '0',
            '-f', 'mpegts', 
            '-flush_packets', '1',  # Force packet flushing
            'udp://239.255.0.1:1234?pkt_size=1316&ttl=0&buffer_size=65536'
        ]

        print("Streamer ffmpeg command:", " ".join(input_cmd))

        try:
            self.process = subprocess.Popen(
                input_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            while self._running:
                if self.process.poll() is not None:
                    break
                line = self.process.stderr.readline()
                if line:
                    print("[FFmpeg stderr]", line, end="")
                self.msleep(50)

        except Exception as e:
            print("StreamerThread error:", e)
        finally:
            if self.process:
                try:
                    self.process.terminate()
                except Exception as e:
                    print("Error terminating streamer process:", e)
                self.process.wait()

    def stop(self):
        self._running = False
        if self.process:
            try:
                self.process.terminate()
            except Exception as e:
                print("Error terminating streamer process in stop():", e)
        self.wait()


# ============================================================
# 3. Thread: Receives the UDP Stream, Decodes, Emits Frames
# ============================================================
class ReceiverThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)
    
    def __init__(self, width=640, height=480, color_mode='color', parent=None):
        super().__init__(parent)
        self.width = width
        self.height = height
        self.color_mode = color_mode
        self._running = True
        self.process = None
        self.data_buffer = bytearray()

    def run(self):
        if self.color_mode == 'bw':
            pix_fmt = 'gray'
            image_format = QImage.Format_Grayscale8
            frame_size = self.width * self.height
        else:
            pix_fmt = 'rgb24'
            image_format = QImage.Format_RGB888
            frame_size = self.width * self.height * 3

        cmd = [
            '-i', 'udp://239.255.0.1:1234?connect=0&reuse=1',
            '-pix_fmt', pix_fmt,
            '-f', 'rawvideo',
            '-'
        ]

        self.process = QProcess()
        print("Receiver ffmpeg command: ffmpeg", " ".join(cmd))
        self.process.start("ffmpeg", cmd)

        if not self.process.waitForStarted(3000):
            print("Receiver ffmpeg process did not start.")
            return

        while self._running:
            if self.process.waitForReadyRead(100):
                chunk = self.process.readAllStandardOutput().data()
                if chunk:
                    self.data_buffer.extend(chunk)

            # If we have enough bytes for a full frame, convert it
            while len(self.data_buffer) >= frame_size:
                frame_bytes = bytes(self.data_buffer[:frame_size])
                del self.data_buffer[:frame_size]
                image = QImage(frame_bytes, self.width, self.height, image_format).copy()
                self.change_pixmap_signal.emit(image)

        if self.process.state() != QProcess.NotRunning:
            self.process.terminate()
            self.process.waitForFinished(3000)

    def stop(self):
        self._running = False
        self.wait()


# ============================================================
# 4. Main Dialog: Single Combo for (Resolution + Framerate)
# ============================================================
class MainDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UDP Streaming Options (Multicast)")
        self.resize(800, 600)

        self.file_path = ""

        # -------------------------------
        # Source selection
        # -------------------------------
        source_layout = QHBoxLayout()

        self.webcam_radio = QRadioButton("Webcam")
        self.file_radio = QRadioButton("Video File")
        self.webcam_radio.setChecked(True)

        self.source_group = QButtonGroup(self)
        self.source_group.addButton(self.webcam_radio)
        self.source_group.addButton(self.file_radio)

        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.browse_file)
        self.webcam_radio.toggled.connect(self.on_source_toggle)

        source_layout.addWidget(self.webcam_radio)
        source_layout.addWidget(self.file_radio)
        source_layout.addWidget(self.browse_button)

        # -------------------------------
        # Options row
        # -------------------------------
        options_layout = QHBoxLayout()

        # Codec
        codec_label = QLabel("Codec:")
        self.codec_combo = QComboBox()
        self.codec_combo.addItems(["H.264", "H.265"])
        options_layout.addWidget(codec_label)
        options_layout.addWidget(self.codec_combo)
        
        # Color mode
        color_label = QLabel("Video Mode:")
        self.color_combo = QComboBox()
        self.color_combo.addItems(["Color", "Black & White"])
        options_layout.addWidget(color_label)
        options_layout.addWidget(self.color_combo)
        
        # Bitrate
        bitrate_label = QLabel("Max Bitrate (kbps):")
        self.bitrate_spin = QSpinBox()
        self.bitrate_spin.setRange(100, 10000)
        self.bitrate_spin.setValue(300)
        options_layout.addWidget(bitrate_label)
        options_layout.addWidget(self.bitrate_spin)
        
        # Audio (only for file)
        self.audio_checkbox = QCheckBox("Audio")
        self.audio_checkbox.setChecked(False)
        self.audio_checkbox.setVisible(False)
        options_layout.addWidget(self.audio_checkbox)
        
        # OSD
        self.osd_checkbox = QCheckBox("OSD")
        self.osd_checkbox.setChecked(True)
        options_layout.addWidget(self.osd_checkbox)
        
        # Apply/Restart
        self.apply_button = QPushButton("Restart Stream")
        self.apply_button.clicked.connect(self.restart_stream)
        options_layout.addWidget(self.apply_button)

        # -------------------------------
        # Webcam layout (single combo)
        # -------------------------------
        webcam_layout = QHBoxLayout()
        self.webcam_mode_label = QLabel("Webcam Modes:")
        self.webcam_mode_combo = QComboBox()

        webcam_layout.addWidget(self.webcam_mode_label)
        webcam_layout.addWidget(self.webcam_mode_combo)

        # -------------------------------
        # Video display
        # -------------------------------
        self.video_label = QLabel()
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black;")
        
        # -------------------------------
        # Main layout
        # -------------------------------
        main_layout = QVBoxLayout(self)
        main_layout.addLayout(source_layout)
        main_layout.addLayout(options_layout)
        main_layout.addLayout(webcam_layout)
        main_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        self.setLayout(main_layout)

        # Populate webcam modes
        self.populate_webcam_modes()

        # Auto-select default file if it exists
        if os.path.exists(VIDEO_FILE_PATH):
            self.file_path = VIDEO_FILE_PATH
            self.file_radio.setChecked(True)
        
        # Start streaming
        self.start_streaming()

    # ---------------------------------
    # Compute aspect ratio helper
    # ---------------------------------
    def compute_aspect_ratio_string(self, w, h):
        gcd_val = math.gcd(w, h)
        ar_w = w // gcd_val
        ar_h = h // gcd_val
        return f"{ar_w}:{ar_h}"

    # ---------------------------------
    # Populate the single webcam combo
    # ---------------------------------
    def populate_webcam_modes(self):
        # If running on Linux but v4l2-ctl is not installed, show warning.
        if sys.platform.startswith('linux'):
            if shutil.which("v4l2-ctl") is None:
                QMessageBox.warning(
                    self,
                    "v4l2-ctl not found",
                    "The 'v4l2-ctl' utility is not installed.\n"
                    "We recommend installing the 'v4l-utils' package for better webcam mode detection."
                )

        self.modes_dict = parse_supported_webcam_modes()

        # If parse failed or returned empty, fallback
        if not self.modes_dict:
            self.modes_dict = {
                "640x480": [30.0],
                "1280x720": [30.0]
            }

        self.webcam_mode_combo.clear()

        # Sort resolutions by width & height
        resolutions = sorted(
            self.modes_dict.keys(),
            key=lambda r: (int(r.split('x')[0]), int(r.split('x')[1]))
        )

        # Fill single combo with "WxH (Aspect) @ FPS"
        # e.g. "640x480 (4:3) @ 30.0 fps"
        for res in resolutions:
            w, h = map(int, res.split('x'))
            aspect = self.compute_aspect_ratio_string(w, h)
            fps_list = self.modes_dict[res]
            # Sort descending by fps
            for f in sorted(fps_list, reverse=True):
                label = f"{res} ({aspect}) @ {f} fps"
                # Store data (width, height, fps) in userData
                self.webcam_mode_combo.addItem(label, (w, h, f))

        # Try to select 1280x720 @ 30 if present
        for i in range(self.webcam_mode_combo.count()):
            data = self.webcam_mode_combo.itemData(i)
            if data is not None:
                w, h, f = data
                if (w == 1280 and h == 720 and abs(f - 30.0) < 0.001):
                    self.webcam_mode_combo.setCurrentIndex(i)
                    break

    # ---------------------------------
    # Source toggles
    # ---------------------------------
    def on_source_toggle(self):
        if self.file_radio.isChecked():
            self.browse_button.setEnabled(True)
            self.audio_checkbox.setVisible(True)
            # Hide webcam modes
            self.webcam_mode_label.setVisible(False)
            self.webcam_mode_combo.setVisible(False)
        else:
            self.browse_button.setEnabled(False)
            self.audio_checkbox.setVisible(False)
            # Show webcam modes
            self.webcam_mode_label.setVisible(True)
            self.webcam_mode_combo.setVisible(True)

    def browse_file(self):
        file_dialog = QFileDialog()
        filename, _ = file_dialog.getOpenFileName(self, "Select Video File")
        if filename:
            self.file_path = filename
            print("Selected file:", self.file_path)

    # ---------------------------------
    # Start / Restart
    # ---------------------------------
    def start_streaming(self):
        if self.file_radio.isChecked() and self.file_path:
            source_type = 'file'
        else:
            source_type = 'webcam'

        codec_text = self.codec_combo.currentText().lower()
        codec = "h265" if "h.265" in codec_text else "h264"
        
        color_text = self.color_combo.currentText().lower()
        color_mode = "bw" if "black" in color_text else "color"
        
        bitrate = self.bitrate_spin.value()
        
        if source_type == 'file':
            audio = self.audio_checkbox.isChecked()
            # Arbitrary default scaling for file
            width, height, fps = 640, 480, 30.0
        else:
            audio = False
            # Retrieve (width, height, fps) from the single combo
            data = self.webcam_mode_combo.currentData()
            if data:
                width, height, fps = data
            else:
                width, height, fps = (640, 480, 30.0)

        osd_enabled = self.osd_checkbox.isChecked()
        
        self.streamer_thread = StreamerThread(
            width=width,
            height=height,
            fps=fps,
            codec=codec,
            color_mode=color_mode,
            bitrate=bitrate,
            source_type=source_type,
            file_path=self.file_path,
            audio=audio,
            osd_enabled=osd_enabled
        )
        self.receiver_thread = ReceiverThread(
            width=width,
            height=height,
            color_mode=color_mode
        )
        self.receiver_thread.change_pixmap_signal.connect(self.update_image)
        
        self.streamer_thread.start()
        self.receiver_thread.start()
    
    def restart_stream(self):
        try:
            if hasattr(self, 'streamer_thread') and self.streamer_thread:
                self.streamer_thread.stop()
        except Exception as e:
            print("Error stopping streamer thread:", e)
        try:
            if hasattr(self, 'receiver_thread') and self.receiver_thread:
                self.receiver_thread.stop()
        except Exception as e:
            print("Error stopping receiver thread:", e)
        
        self.start_streaming()
    
    # ---------------------------------
    # Update display
    # ---------------------------------
    def update_image(self, image):
        self.video_label.setPixmap(QPixmap.fromImage(image))
    
    # ---------------------------------
    # Cleanup
    # ---------------------------------
    def closeEvent(self, event):
        try:
            if hasattr(self, 'streamer_thread') and self.streamer_thread:
                self.streamer_thread.stop()
        except Exception:
            pass
        try:
            if hasattr(self, 'receiver_thread') and self.receiver_thread:
                self.receiver_thread.stop()
        except Exception:
            pass
        event.accept()


# ============================================================
# Main Entry
# ============================================================
if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = MainDialog()
    dialog.show()
    sys.exit(app.exec_())
