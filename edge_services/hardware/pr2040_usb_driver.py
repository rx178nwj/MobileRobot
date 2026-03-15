#!/usr/bin/env python3
"""
PR2040 Motor Driver USB Serial Interface

Based on CommandReference.md
Protocol: [0xAA][CMD][LEN][DATA...][CHECKSUM]
"""

import serial
import struct
import logging
from typing import Tuple, Optional
import time


class PR2040USBDriver:
    """PR2040 Motor Driver USB Serial Interface"""

    PACKET_HEADER = 0xAA

    # Commands
    CMD_SET_MOTORS = 0x01
    CMD_STOP_ALL = 0x02
    CMD_BRAKE_ALL = 0x03
    CMD_SET_MOTOR_SINGLE = 0x04
    CMD_REQUEST_STATUS = 0x10
    CMD_RESET_ENCODERS = 0x11
    CMD_REQUEST_EXT_ADC = 0x12
    CMD_SET_MODE_ALL = 0x20
    CMD_SET_MODE_SINGLE = 0x21
    CMD_SET_VEL_ALL = 0x22
    CMD_SET_VEL_SINGLE = 0x23
    CMD_SET_POS_ALL = 0x24
    CMD_SET_POS_SINGLE = 0x25
    CMD_REQUEST_TEMP = 0x15

    # Responses
    RESP_ACK = 0x80
    RESP_NAK = 0x81
    RESP_STATUS = 0x91
    RESP_EXT_ADC = 0x92
    RESP_IMU = 0x93
    RESP_TEMP = 0x94

    # Control Modes
    MODE_DIRECT = 0
    MODE_VELOCITY = 1
    MODE_POSITION = 2

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        """
        Initialize USB serial connection

        Args:
            port: Serial port path
            baudrate: Baud rate (default: 115200)
        """
        self.logger = logging.getLogger(__name__)
        self.port = port
        self.baudrate = baudrate
        self.connected = False

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            self.logger.info(f"PR2040 USB Driver connected at {port}")

            # Set VELOCITY mode by default
            self._set_mode_all(self.MODE_VELOCITY)

        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to PR2040: {e}")
            self.connected = False

    def close(self):
        """Close serial connection"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.stop_all()
            self.ser.close()
            self.connected = False
            self.logger.info("PR2040 USB connection closed")

    def _calc_checksum(self, cmd: int, length: int, data: bytes) -> int:
        """Calculate XOR checksum"""
        cs = cmd ^ length
        for b in data:
            cs ^= b
        return cs

    def _send_cmd(self, cmd: int, data: bytes = b''):
        """Send command packet"""
        if not self.connected:
            return False

        length = len(data)
        checksum = self._calc_checksum(cmd, length, data)
        packet = bytes([self.PACKET_HEADER, cmd, length]) + data + bytes([checksum])

        try:
            self.ser.write(packet)
            # Drain status packets
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            self.logger.error(f"Send command error: {e}")
            return False

    def _read_status_packet(self) -> Optional[dict]:
        """Read and parse STATUS packet"""
        if not self.connected:
            return None

        try:
            # Wait for data
            timeout = time.time() + 0.2
            while self.ser.in_waiting < 48 and time.time() < timeout:
                time.sleep(0.01)

            if self.ser.in_waiting < 48:
                return None

            # Find STATUS packet
            while self.ser.in_waiting >= 48:
                b = self.ser.read(1)
                if len(b) == 0 or b[0] != self.PACKET_HEADER:
                    continue

                type_byte = self.ser.read(1)
                if len(type_byte) == 0:
                    continue

                type_val = type_byte[0]
                length = self.ser.read(1)[0]

                if type_val == self.RESP_STATUS and length == 44:
                    data = self.ser.read(44)
                    checksum = self.ser.read(1)[0]

                    # Verify checksum
                    calc_cs = self._calc_checksum(type_val, length, data)
                    if calc_cs == checksum:
                        # Parse status
                        encoders = struct.unpack_from('<iiii', data, 0)
                        velocities = struct.unpack_from('<iiii', data, 16)
                        adc = struct.unpack_from('<hhhh', data, 32)
                        timestamp = struct.unpack_from('<I', data, 40)[0]

                        return {
                            'encoders': encoders,
                            'velocities': velocities,
                            'adc': adc,
                            'timestamp': timestamp
                        }
                else:
                    # Skip packet
                    if length > 0:
                        self.ser.read(length + 1)

        except Exception as e:
            self.logger.error(f"Read status error: {e}")

        return None

    # Public API methods

    def stop_all(self):
        """Stop all motors (also sets DIRECT mode)"""
        return self._send_cmd(self.CMD_STOP_ALL)

    def reset_encoders(self):
        """Reset all encoder counts to zero"""
        return self._send_cmd(self.CMD_RESET_ENCODERS)

    def _set_mode_all(self, mode: int):
        """Set control mode for all motors"""
        return self._send_cmd(self.CMD_SET_MODE_ALL, bytes([mode, mode, mode, mode]))

    def set_wheel_velocities(self, velocities: Tuple[float, float, float, float]):
        """
        Set velocity targets for all 4 wheels (counts/sec)

        Args:
            velocities: Tuple of 4 float values (counts/sec)
        """
        if not self.connected:
            return

        data = struct.pack('<ffff', *velocities)
        self._send_cmd(self.CMD_SET_VEL_ALL, data)

    def read_all_encoders(self) -> Tuple[int, int, int, int]:
        """
        Read all encoder counts

        Returns:
            Tuple of 4 encoder counts
        """
        if not self.connected:
            return (0, 0, 0, 0)

        # Request status
        self.ser.reset_input_buffer()
        self._send_cmd(self.CMD_REQUEST_STATUS)

        # Read status packet
        status = self._read_status_packet()
        if status:
            return status['encoders']

        return (0, 0, 0, 0)

    def read_encoder(self, index: int) -> int:
        """
        Read single encoder count

        Args:
            index: Encoder index (0-3)

        Returns:
            Encoder count
        """
        encoders = self.read_all_encoders()
        if 0 <= index < 4:
            return encoders[index]
        return 0

    def read_temperature(self) -> float:
        """
        Read board temperature

        Returns:
            Temperature in Celsius
        """
        if not self.connected:
            return 0.0

        try:
            self.ser.reset_input_buffer()
            self._send_cmd(self.CMD_REQUEST_TEMP)
            time.sleep(0.05)

            # Read response
            if self.ser.in_waiting >= 6:
                header = self.ser.read(1)[0]
                if header == self.PACKET_HEADER:
                    resp_type = self.ser.read(1)[0]
                    length = self.ser.read(1)[0]

                    if resp_type == self.RESP_TEMP and length == 4:
                        data = self.ser.read(4)
                        checksum = self.ser.read(1)[0]

                        calc_cs = self._calc_checksum(resp_type, length, data)
                        if calc_cs == checksum:
                            temp = struct.unpack('<f', data)[0]
                            return temp

        except Exception as e:
            self.logger.error(f"Read temperature error: {e}")

        return 0.0

    def get_status(self) -> Optional[dict]:
        """
        Get full status

        Returns:
            Dictionary with encoders, velocities, adc, timestamp
        """
        if not self.connected:
            return None

        self.ser.reset_input_buffer()
        self._send_cmd(self.CMD_REQUEST_STATUS)
        return self._read_status_packet()
