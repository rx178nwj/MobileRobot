#!/usr/bin/env python3
"""
PR2040 Motor Driver I2C Communication Library

Hardware: PR2040_BASESYSTEM REV1.5.0 (N.W Works)
I2C Address: 0x60
Interface: I2C (smbus2)

Features:
- 4-channel motor control (velocity mode)
- 4-channel encoder reading
- Temperature sensor reading
- IMU data reading (future)
"""

import struct
import logging
from typing import Tuple, List, Optional
import time

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    logging.warning("smbus2 not available - running in dummy mode")


class PR2040Driver:
    """PR2040 Motor Driver I2C Interface"""

    # I2C Address
    I2C_ADDRESS = 0x60
    I2C_BUS = 1  # I2C bus number (typically 1 on Raspberry Pi)

    # Register Map - Write
    REG_MOTOR_ALL = 0x01        # 8B: int16 x4 motor duties
    REG_MOTOR_SINGLE = 0x02     # 3B: uint8 index + int16 duty
    REG_STOP_ALL = 0x03         # 0B: coast all motors
    REG_BRAKE_ALL = 0x04        # 0B: brake all motors
    REG_RESET_ENC = 0x05        # 0B: reset encoders
    REG_SET_MODE_ALL = 0x06     # 4B: uint8 x4 control modes
    REG_SET_MODE_SINGLE = 0x07  # 2B: uint8 index + uint8 mode
    REG_SET_VEL_ALL = 0x08      # 16B: float x4 velocity targets (cps)
    REG_SET_VEL_SINGLE = 0x09   # 5B: uint8 index + float target
    REG_SET_POS_ALL = 0x0A      # 16B: int32 x4 position targets
    REG_SET_POS_SINGLE = 0x0B   # 5B: uint8 index + int32 target
    REG_SET_VEL_PID = 0x0C      # 13B: uint8 index + float kp,ki,kd
    REG_SET_POS_GAINS = 0x0D    # 9B: uint8 index + float posKp,maxVelCps

    # Register Map - Read
    REG_ENC0 = 0x10             # 4B: int32 encoder 0
    REG_ENC1 = 0x11             # 4B: int32 encoder 1
    REG_ENC2 = 0x12             # 4B: int32 encoder 2
    REG_ENC3 = 0x13             # 4B: int32 encoder 3
    REG_VEL0 = 0x14             # 4B: int32 velocity 0 (cps)
    REG_VEL1 = 0x15             # 4B: int32 velocity 1
    REG_VEL2 = 0x16             # 4B: int32 velocity 2
    REG_VEL3 = 0x17             # 4B: int32 velocity 3
    REG_ADC0 = 0x20             # 2B: int16 ADC ch0
    REG_STATUS = 0x30           # 44B: full status
    REG_BOARD_TEMP = 0x60       # 4B: float board temperature (°C)
    REG_DEVICE_ID = 0xFF        # 2B: device ID (0x20, 0x40)

    # Control Modes
    MODE_DIRECT = 0    # Direct duty control (-1000 to +1000)
    MODE_VELOCITY = 1  # Velocity PID control (counts/sec)
    MODE_POSITION = 2  # Position cascade control (counts)

    def __init__(self, bus_number: int = I2C_BUS, address: int = I2C_ADDRESS):
        """
        Initialize PR2040 driver

        Args:
            bus_number: I2C bus number (default: 1)
            address: I2C slave address (default: 0x60)
        """
        self.bus_number = bus_number
        self.address = address
        self.logger = logging.getLogger(__name__)

        self.connected = False  # Initialize before attempting connection

        if SMBUS_AVAILABLE:
            try:
                self.bus = smbus2.SMBus(bus_number)
                self.connected = self._verify_connection()
                if self.connected:
                    self.logger.info(f"PR2040 Driver connected at 0x{address:02X}")
                    self._set_velocity_mode()
                else:
                    self.logger.warning("PR2040 Driver not responding - running in dummy mode")
            except Exception as e:
                self.logger.error(f"Failed to initialize I2C: {e}")
                self.connected = False
        else:
            self.bus = None
            self.connected = False
            self.logger.warning("Running in dummy mode (smbus2 not available)")

    def _verify_connection(self) -> bool:
        """Verify connection by reading device ID"""
        try:
            device_id = self._read_register(self.REG_DEVICE_ID, 2)
            if device_id == bytes([0x20, 0x40]):
                return True
            else:
                self.logger.warning(f"Unexpected device ID: {device_id.hex()}")
                return False
        except Exception as e:
            self.logger.error(f"Connection verification failed: {e}")
            return False

    def _set_velocity_mode(self):
        """Set all motors to velocity control mode"""
        try:
            # Set all 4 motors to velocity mode (MODE_VELOCITY = 1)
            data = [self.MODE_VELOCITY] * 4
            self._write_register(self.REG_SET_MODE_ALL, data)
            self.logger.info("Motors set to velocity control mode")
        except Exception as e:
            self.logger.error(f"Failed to set velocity mode: {e}")

    def _read_register(self, register: int, length: int) -> bytes:
        """
        Read data from I2C register

        Args:
            register: Register address
            length: Number of bytes to read

        Returns:
            bytes: Read data
        """
        if not self.connected or self.bus is None:
            return bytes(length)  # Return dummy data

        try:
            self.bus.write_byte(self.address, register)
            time.sleep(0.001)  # Small delay
            data = self.bus.read_i2c_block_data(self.address, register, length)
            return bytes(data)
        except Exception as e:
            self.logger.error(f"I2C read error at 0x{register:02X}: {e}")
            return bytes(length)

    def _write_register(self, register: int, data: List[int]):
        """
        Write data to I2C register

        Args:
            register: Register address
            data: Data bytes to write
        """
        if not self.connected or self.bus is None:
            return  # Skip in dummy mode

        try:
            self.bus.write_i2c_block_data(self.address, register, data)
        except Exception as e:
            self.logger.error(f"I2C write error at 0x{register:02X}: {e}")

    # ===== Motor Control Methods =====

    def set_wheel_velocities(self, velocities: Tuple[float, float, float, float]):
        """
        Set all 4 wheel velocities (counts/sec)

        Args:
            velocities: Tuple of (wheel0, wheel1, wheel2, wheel3) velocities in cps
        """
        data = struct.pack('<ffff', *velocities)
        self._write_register(self.REG_SET_VEL_ALL, list(data))

    def set_single_velocity(self, index: int, velocity: float):
        """
        Set single wheel velocity

        Args:
            index: Wheel index (0-3)
            velocity: Velocity in counts/sec
        """
        data = struct.pack('<Bf', index, velocity)
        self._write_register(self.REG_SET_VEL_SINGLE, list(data))

    def stop_all(self):
        """Coast stop all motors"""
        self._write_register(self.REG_STOP_ALL, [])
        self.logger.info("All motors stopped (coast)")

    def brake_all(self):
        """Active brake all motors"""
        self._write_register(self.REG_BRAKE_ALL, [])
        self.logger.info("All motors braked")

    # ===== Encoder Reading Methods =====

    def read_encoder(self, index: int) -> int:
        """
        Read single encoder count

        Args:
            index: Encoder index (0-3)

        Returns:
            int: Encoder count (int32)
        """
        register = self.REG_ENC0 + index
        data = self._read_register(register, 4)
        return struct.unpack('<i', data)[0]

    def read_all_encoders(self) -> Tuple[int, int, int, int]:
        """
        Read all 4 encoder counts

        Returns:
            Tuple of (enc0, enc1, enc2, enc3) counts
        """
        encoders = []
        for i in range(4):
            encoders.append(self.read_encoder(i))
        return tuple(encoders)

    def read_velocity(self, index: int) -> int:
        """
        Read single wheel velocity

        Args:
            index: Wheel index (0-3)

        Returns:
            int: Velocity in counts/sec (int32)
        """
        register = self.REG_VEL0 + index
        data = self._read_register(register, 4)
        return struct.unpack('<i', data)[0]

    def read_all_velocities(self) -> Tuple[int, int, int, int]:
        """
        Read all 4 wheel velocities

        Returns:
            Tuple of (vel0, vel1, vel2, vel3) in counts/sec
        """
        velocities = []
        for i in range(4):
            velocities.append(self.read_velocity(i))
        return tuple(velocities)

    def reset_encoders(self):
        """Reset all encoder counts to zero"""
        self._write_register(self.REG_RESET_ENC, [])
        self.logger.info("Encoders reset")

    # ===== Sensor Reading Methods =====

    def read_temperature(self) -> float:
        """
        Read board temperature from S-5851A sensor

        Returns:
            float: Temperature in Celsius
        """
        data = self._read_register(self.REG_BOARD_TEMP, 4)
        return struct.unpack('<f', data)[0]

    def read_status(self) -> dict:
        """
        Read full status (44 bytes)

        Returns:
            dict: Status data containing encoders, velocities, ADCs, timestamp
        """
        data = self._read_register(self.REG_STATUS, 44)

        # Parse status data
        # [0..15]  int32 x4  encoder counts
        # [16..31] int32 x4  velocities (cps)
        # [32..39] int16 x4  ADC raw (0..4095)
        # [40..43] uint32    timestamp (ms)

        encoders = struct.unpack('<iiii', data[0:16])
        velocities = struct.unpack('<iiii', data[16:32])
        adcs = struct.unpack('<hhhh', data[32:40])
        timestamp = struct.unpack('<I', data[40:44])[0]

        return {
            'encoders': encoders,
            'velocities': velocities,
            'adcs': adcs,
            'timestamp': timestamp
        }

    def close(self):
        """Close I2C connection"""
        if self.bus is not None:
            self.bus.close()
            self.logger.info("I2C bus closed")


if __name__ == '__main__':
    # Test code
    logging.basicConfig(level=logging.INFO)

    driver = PR2040Driver()

    if driver.connected:
        print("Testing PR2040 Driver...")

        # Read device info
        print(f"Temperature: {driver.read_temperature():.2f}°C")

        # Read encoders
        encoders = driver.read_all_encoders()
        print(f"Encoders: {encoders}")

        # Read velocities
        velocities = driver.read_all_velocities()
        print(f"Velocities: {velocities}")

        # Read full status
        status = driver.read_status()
        print(f"Status: {status}")

        driver.close()
    else:
        print("Driver not connected - check I2C connection")
