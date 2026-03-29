#!/usr/bin/env python3
"""
PR2040 Motor Driver USB Serial Interface

Based on CommandReference.md
Protocol: [0xAA][CMD][LEN][DATA...][CHECKSUM]

Architecture:
- Background reader thread continuously parses 100Hz auto-status packets
- Latest status cached in memory (encoders, velocities)
- Motor commands written directly to serial (separate write lock)
- No blocking sleeps in the hot path
"""

import serial
import struct
import logging
import threading
from typing import Tuple, Optional
import time


class PR2040USBDriver:
    """PR2040 Motor Driver USB Serial Interface"""

    PACKET_HEADER = 0xAA

    # Commands
    CMD_SET_MOTORS      = 0x01
    CMD_STOP_ALL        = 0x02
    CMD_BRAKE_ALL       = 0x03
    CMD_SET_MOTOR_SINGLE = 0x04
    CMD_REQUEST_STATUS  = 0x10
    CMD_RESET_ENCODERS  = 0x11
    CMD_REQUEST_EXT_ADC = 0x12
    CMD_REQUEST_TEMP    = 0x15
    CMD_SET_MODE_ALL    = 0x20
    CMD_SET_MODE_SINGLE = 0x21
    CMD_SET_VEL_ALL     = 0x22
    CMD_SET_VEL_SINGLE  = 0x23
    CMD_SET_POS_ALL     = 0x24
    CMD_SET_POS_SINGLE  = 0x25
    CMD_SET_VEL_PID     = 0x26  # 13B: uint8 index + float kp, ki, kd

    # Responses
    RESP_ACK    = 0x80
    RESP_NAK    = 0x81
    RESP_STATUS = 0x91
    RESP_EXT_ADC = 0x92
    RESP_IMU    = 0x93
    RESP_TEMP   = 0x94

    # Control Modes
    MODE_DIRECT   = 0
    MODE_VELOCITY = 1
    MODE_POSITION = 2

    # RESP_STATUS packet size (header + type + len + 44 data + checksum)
    STATUS_DATA_LEN = 44

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        self.logger = logging.getLogger(__name__)
        self.port = port
        self.baudrate = baudrate
        self.connected = False

        # Cached status (updated by reader thread)
        self._status_lock = threading.Lock()
        self._latest_status = {
            'encoders': (0, 0, 0, 0),
            'velocities': (0, 0, 0, 0),
            'adc': (0, 0, 0, 0),
            'timestamp': 0,
        }
        self._status_updated = False

        # Write lock (only one command at a time)
        self._write_lock = threading.Lock()

        # Reader thread
        self._reader_thread = None
        self._stop_reader = threading.Event()

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            self.logger.info(f"PR2040 USB Driver connected at {port}")

            # Start background reader
            self._reader_thread = threading.Thread(
                target=self._reader_loop, daemon=True, name='PR2040Reader'
            )
            self._reader_thread.start()

            # Wait for first status packet
            deadline = time.time() + 2.0
            while not self._status_updated and time.time() < deadline:
                time.sleep(0.05)

            # Control mode: MODE_DIRECT  = open-loop duty (Python-side PID)
            #               MODE_VELOCITY = firmware velocity PID
            # Default is DIRECT so the Python-side PI+FF controller drives the motors.
            self._set_mode_all(self.MODE_DIRECT)

        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to PR2040: {e}")
            self.connected = False

    # ------------------------------------------------------------------
    # Background reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self):
        """Continuously read and parse incoming packets from PR2040."""
        buf = bytearray()
        while not self._stop_reader.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Process all complete packets in buffer
                while True:
                    # Find header
                    idx = buf.find(self.PACKET_HEADER)
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]

                    # Need at least 3 bytes (header + type + len)
                    if len(buf) < 3:
                        break

                    pkt_type = buf[1]
                    data_len = buf[2]
                    total_len = 3 + data_len + 1  # header+type+len + data + checksum

                    if len(buf) < total_len:
                        break  # Wait for more data

                    data = bytes(buf[3:3 + data_len])
                    checksum = buf[3 + data_len]
                    del buf[:total_len]

                    # Verify checksum
                    cs = pkt_type ^ data_len
                    for b in data:
                        cs ^= b
                    if cs != checksum:
                        continue

                    # Parse STATUS packet
                    if pkt_type == self.RESP_STATUS and data_len == self.STATUS_DATA_LEN:
                        encoders   = struct.unpack_from('<iiii', data, 0)
                        velocities = struct.unpack_from('<iiii', data, 16)
                        adc        = struct.unpack_from('<hhhh', data, 32)
                        timestamp  = struct.unpack_from('<I',    data, 40)[0]
                        with self._status_lock:
                            self._latest_status = {
                                'encoders': encoders,
                                'velocities': velocities,
                                'adc': adc,
                                'timestamp': timestamp,
                            }
                            self._status_updated = True

            except Exception as e:
                if not self._stop_reader.is_set():
                    self.logger.error(f"Reader error: {e}")
                break

    # ------------------------------------------------------------------
    # Command sending (write lock only, no sleep)
    # ------------------------------------------------------------------

    def _calc_checksum(self, cmd: int, length: int, data: bytes) -> int:
        cs = cmd ^ length
        for b in data:
            cs ^= b
        return cs

    def _send_cmd(self, cmd: int, data: bytes = b'') -> bool:
        if not self.connected:
            return False
        length = len(data)
        checksum = self._calc_checksum(cmd, length, data)
        packet = bytes([self.PACKET_HEADER, cmd, length]) + data + bytes([checksum])
        with self._write_lock:
            try:
                self.ser.write(packet)
                return True
            except Exception as e:
                self.logger.error(f"Send command error: {e}")
                return False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def close(self):
        self._stop_reader.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.stop_all()
            self.ser.close()
        self.connected = False
        self.logger.info("PR2040 USB connection closed")

    def stop_all(self) -> bool:
        return self._send_cmd(self.CMD_STOP_ALL)

    def reset_encoders(self) -> bool:
        return self._send_cmd(self.CMD_RESET_ENCODERS)

    def _set_mode_all(self, mode: int) -> bool:
        return self._send_cmd(self.CMD_SET_MODE_ALL, bytes([mode, mode, mode, mode]))

    def set_wheel_velocities(self, velocities: Tuple[float, float, float, float]) -> bool:
        """Set velocity targets [counts/sec] for all 4 wheels (VELOCITY mode)."""
        if not self.connected:
            return False
        data = struct.pack('<ffff', *velocities)
        return self._send_cmd(self.CMD_SET_VEL_ALL, data)

    def set_velocity_pid_gains(self, index: int, kp: float, ki: float, kd: float) -> bool:
        """
        Set velocity PID gains for one wheel (firmware VELOCITY mode).

        Args:
            index: Wheel index (0-3)
            kp, ki, kd: PID gains

        Note: Only effective when MODE_VELOCITY is active.
              Call _set_mode_all(MODE_VELOCITY) first.
        """
        if not self.connected:
            return False
        data = struct.pack('<Bfff', index, kp, ki, kd)
        return self._send_cmd(self.CMD_SET_VEL_PID, data)

    def set_wheel_duties(self, duties: Tuple[int, int, int, int]) -> bool:
        """Set duty cycles directly for all 4 wheels (-1000 to +1000, DIRECT mode)."""
        if not self.connected:
            return False
        clamped = tuple(max(-1000, min(1000, int(d))) for d in duties)
        data = struct.pack('<hhhh', *clamped)
        return self._send_cmd(self.CMD_SET_MOTORS, data)

    def read_all_encoders(self) -> Tuple[int, int, int, int]:
        """Return latest cached encoder counts (no serial round-trip)."""
        with self._status_lock:
            return self._latest_status['encoders']

    def read_encoder(self, index: int) -> int:
        return self.read_all_encoders()[index] if 0 <= index < 4 else 0

    def get_status(self) -> dict:
        """Return latest cached full status."""
        with self._status_lock:
            return dict(self._latest_status)

    def read_temperature(self) -> float:
        """Temperature is not in auto-status; return 0.0 placeholder."""
        return 0.0
