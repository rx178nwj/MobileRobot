#!/usr/bin/env python3
"""
PR2040 USB Serial Test (Corrected Protocol)

Protocol: [0xAA][CMD][LEN][DATA...][CHECKSUM]
CHECKSUM = XOR(CMD, LEN, DATA[0..LEN-1])
"""

import serial
import struct
import time
import sys


class PR2040USB:
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
    CMD_REQUEST_TEMP = 0x15

    # Responses
    RESP_ACK = 0x80
    RESP_NAK = 0x81
    RESP_STATUS = 0x91
    RESP_EXT_ADC = 0x92
    RESP_IMU = 0x93
    RESP_TEMP = 0x94

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1.0):
        """Initialize USB serial connection"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"✅ Connected to PR2040 at {port}")

            # Clear any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        except serial.SerialException as e:
            print(f"❌ Failed to connect: {e}")
            sys.exit(1)

    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Serial connection closed")

    def _calc_checksum(self, cmd, length, data):
        """Calculate XOR checksum"""
        cs = cmd ^ length
        for b in data:
            cs ^= b
        return cs

    def _send_packet(self, cmd, data=b''):
        """Send packet to PR2040"""
        length = len(data)
        checksum = self._calc_checksum(cmd, length, data)

        packet = bytes([self.PACKET_HEADER, cmd, length]) + data + bytes([checksum])
        self.ser.write(packet)

    def _read_response(self, expected_resp=None, timeout=0.5):
        """Read response packet"""
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting >= 3:  # Minimum: header + cmd + len
                # Read header
                header = self.ser.read(1)[0]
                if header != self.PACKET_HEADER:
                    continue

                # Read cmd and length
                cmd = self.ser.read(1)[0]
                length = self.ser.read(1)[0]

                # Wait for data + checksum
                while self.ser.in_waiting < (length + 1):
                    time.sleep(0.01)
                    if (time.time() - start_time) > timeout:
                        return None

                # Read data and checksum
                data = self.ser.read(length) if length > 0 else b''
                checksum = self.ser.read(1)[0]

                # Verify checksum
                calc_cs = self._calc_checksum(cmd, length, data)
                if calc_cs != checksum:
                    print(f"⚠️  Checksum mismatch: got {checksum:02X}, expected {calc_cs:02X}")
                    continue

                # Check if expected response
                if expected_resp is not None and cmd != expected_resp:
                    print(f"⚠️  Unexpected response: got {cmd:02X}, expected {expected_resp:02X}")
                    continue

                return (cmd, data)

            time.sleep(0.01)

        return None

    def request_temperature(self):
        """Request board temperature"""
        self._send_packet(self.CMD_REQUEST_TEMP)
        response = self._read_response(self.RESP_TEMP)

        if response:
            cmd, data = response
            if len(data) == 4:
                temp = struct.unpack('<f', data)[0]
                return temp

        return None

    def request_status(self):
        """Request full status"""
        self._send_packet(self.CMD_REQUEST_STATUS)
        response = self._read_response(self.RESP_STATUS)

        if response:
            cmd, data = response
            if len(data) >= 44:
                # Parse: enc(int32x4) + vel(int32x4) + adc(int16x4) + ts(uint32)
                encoders = struct.unpack('<iiii', data[0:16])
                velocities = struct.unpack('<iiii', data[16:32])
                adc = struct.unpack('<hhhh', data[32:40])
                timestamp = struct.unpack('<I', data[40:44])[0]

                return {
                    'encoders': encoders,
                    'velocities': velocities,
                    'adc': adc,
                    'timestamp': timestamp
                }

        return None

    def reset_encoders(self):
        """Reset all encoder counts"""
        self._send_packet(self.CMD_RESET_ENCODERS)
        response = self._read_response(self.RESP_ACK)
        return response is not None

    def stop_all(self):
        """Stop all motors (coast)"""
        self._send_packet(self.CMD_STOP_ALL)
        response = self._read_response(self.RESP_ACK)
        return response is not None

    def set_velocity_all(self, v0, v1, v2, v3):
        """Set velocity for all 4 motors (counts/sec)"""
        data = struct.pack('<ffff', float(v0), float(v1), float(v2), float(v3))
        self._send_packet(self.CMD_SET_VEL_ALL, data)
        response = self._read_response(self.RESP_ACK)
        return response is not None


def test_basic():
    """Test basic communication"""
    print("=" * 60)
    print("PR2040 USB Serial - Basic Communication Test")
    print("=" * 60)

    pr2040 = PR2040USB()

    try:
        # Test 1: Temperature
        print("\n[Test 1] Reading board temperature...")
        temp = pr2040.request_temperature()
        if temp is not None:
            print(f"✅ Temperature: {temp:.2f}°C")
        else:
            print("❌ Failed to read temperature")

        # Test 2: Status
        print("\n[Test 2] Reading status...")
        status = pr2040.request_status()
        if status:
            print(f"✅ Encoders: {status['encoders']}")
            print(f"✅ Velocities: {status['velocities']}")
            print(f"✅ ADC: {status['adc']}")
            print(f"✅ Timestamp: {status['timestamp']} ms")
        else:
            print("❌ Failed to read status")

        # Test 3: Reset encoders
        print("\n[Test 3] Resetting encoders...")
        if pr2040.reset_encoders():
            print("✅ Encoders reset")
            time.sleep(0.2)
            status = pr2040.request_status()
            if status:
                print(f"   New values: {status['encoders']}")
        else:
            print("❌ Failed to reset encoders")

        print("\n" + "=" * 60)
        print("Basic tests completed!")
        print("=" * 60)

    finally:
        pr2040.close()


def test_move_10cm():
    """Test: Move forward 10cm"""
    print("=" * 60)
    print("PR2040 Motor Movement Test - Forward 10cm")
    print("=" * 60)

    pr2040 = PR2040USB()

    try:
        # Robot parameters
        WHEEL_RADIUS = 0.033  # meters
        ENCODER_CPR = 720     # counts per revolution
        WHEEL_CIRCUMFERENCE = 2 * 3.14159 * WHEEL_RADIUS
        METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / ENCODER_CPR

        target_distance = 0.10  # 10cm
        target_counts = int(target_distance / METERS_PER_COUNT)
        target_velocity = 100.0  # counts/sec (slow)
        duration = target_counts / target_velocity

        print(f"\nTarget: {target_distance*100}cm = {target_counts} counts")
        print(f"Velocity: {target_velocity} counts/sec")
        print(f"Duration: {duration:.1f} seconds")

        # Reset encoders
        print("\n[1/5] Resetting encoders...")
        pr2040.reset_encoders()
        time.sleep(0.2)

        # Check initial
        print("[2/5] Reading initial position...")
        status = pr2040.request_status()
        if status:
            print(f"   Initial: {status['encoders']}")

        # Start movement
        print(f"\n[3/5] Moving forward at {target_velocity} cps...")
        pr2040.set_velocity_all(target_velocity, target_velocity,
                               target_velocity, target_velocity)

        # Monitor
        print("[4/5] Monitoring...")
        start_time = time.time()

        while (time.time() - start_time) < duration:
            time.sleep(0.5)
            status = pr2040.request_status()
            if status:
                avg_enc = sum(status['encoders']) / 4.0
                elapsed = time.time() - start_time
                print(f"   t={elapsed:.1f}s: avg={avg_enc:.0f} counts")

        # Stop
        print("\n[5/5] Stopping...")
        pr2040.stop_all()
        time.sleep(0.2)

        # Final
        status = pr2040.request_status()
        if status:
            avg_final = sum(status['encoders']) / 4.0
            distance = avg_final * METERS_PER_COUNT

            print(f"\n📊 Results:")
            print(f"   Final: {status['encoders']}")
            print(f"   Average: {avg_final:.0f} counts")
            print(f"   Distance: {distance*100:.1f} cm")

        print("\n" + "=" * 60)
        print("Movement test completed!")
        print("=" * 60)

    finally:
        pr2040.stop_all()
        pr2040.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='PR2040 USB Test')
    parser.add_argument('--test', choices=['basic', 'move'], default='basic')
    args = parser.parse_args()

    if args.test == 'basic':
        test_basic()
    elif args.test == 'move':
        test_move_10cm()
