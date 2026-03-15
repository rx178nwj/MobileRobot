#!/usr/bin/env python3
"""
PR2040 USB Serial Test

Tests communication with PR2040 motor driver via USB Serial.
This allows direct control without I2C.
"""

import serial
import struct
import time
import sys


class PR2040USB:
    """PR2040 Motor Driver USB Serial Interface"""

    # USB Serial Commands (from PR2040_MotorDriver.ino)
    CMD_REQUEST_STATUS = 0x10
    CMD_REQUEST_TEMP = 0x15
    CMD_SET_VEL_ALL = 0x20
    CMD_STOP_ALL = 0x30
    CMD_RESET_ENC = 0x40

    RESP_STATUS = 0x90
    RESP_TEMP = 0x94
    RESP_ACK = 0xA0

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

    def request_status(self):
        """Request full status from PR2040"""
        # Send command
        self.ser.write(bytes([self.CMD_REQUEST_STATUS]))

        # Wait for response
        time.sleep(0.1)

        if self.ser.in_waiting < 45:  # Header(1) + Status(44)
            print("⚠️  No status response")
            return None

        # Read response
        header = self.ser.read(1)[0]
        if header != self.RESP_STATUS:
            print(f"⚠️  Unexpected response header: 0x{header:02X}")
            return None

        # Read 44 bytes status
        data = self.ser.read(44)

        # Parse status (4 encoders + 4 velocities + 8 ADC channels)
        encoders = struct.unpack('<iiii', data[0:16])
        velocities = struct.unpack('<iiii', data[16:32])
        adc = struct.unpack('<hhhhhhhh', data[32:48])

        return {
            'encoders': encoders,
            'velocities': velocities,
            'adc': adc
        }

    def request_temperature(self):
        """Request board temperature"""
        # Send command
        self.ser.write(bytes([self.CMD_REQUEST_TEMP]))

        # Wait for response
        time.sleep(0.1)

        if self.ser.in_waiting < 5:  # Header(1) + float(4)
            print("⚠️  No temperature response")
            return None

        # Read response
        header = self.ser.read(1)[0]
        if header != self.RESP_TEMP:
            print(f"⚠️  Unexpected response header: 0x{header:02X}")
            return None

        # Read temperature (float, 4 bytes)
        temp_bytes = self.ser.read(4)
        temperature = struct.unpack('<f', temp_bytes)[0]

        return temperature

    def set_velocity_all(self, vel0, vel1, vel2, vel3):
        """
        Set velocity for all 4 motors

        Args:
            vel0-vel3: Target velocities in counts/sec
        """
        # Prepare command
        cmd = struct.pack('<Bffff',
            self.CMD_SET_VEL_ALL,
            float(vel0),
            float(vel1),
            float(vel2),
            float(vel3)
        )

        # Send command
        self.ser.write(cmd)

        # Wait for ACK
        time.sleep(0.05)

        if self.ser.in_waiting > 0:
            header = self.ser.read(1)[0]
            if header == self.RESP_ACK:
                return True

        return False

    def stop_all(self):
        """Stop all motors"""
        self.ser.write(bytes([self.CMD_STOP_ALL]))
        time.sleep(0.05)

        if self.ser.in_waiting > 0:
            header = self.ser.read(1)[0]
            return header == self.RESP_ACK

        return False

    def reset_encoders(self):
        """Reset all encoder counts to zero"""
        self.ser.write(bytes([self.CMD_RESET_ENC]))
        time.sleep(0.05)

        if self.ser.in_waiting > 0:
            header = self.ser.read(1)[0]
            return header == self.RESP_ACK

        return False


def test_basic_communication():
    """Test basic USB communication"""
    print("=" * 60)
    print("PR2040 USB Serial Communication Test")
    print("=" * 60)

    pr2040 = PR2040USB()

    try:
        # Test 1: Temperature reading
        print("\n[Test 1] Reading board temperature...")
        temp = pr2040.request_temperature()
        if temp is not None:
            print(f"✅ Temperature: {temp:.2f}°C")
        else:
            print("❌ Failed to read temperature")

        # Test 2: Status reading
        print("\n[Test 2] Reading status...")
        status = pr2040.request_status()
        if status:
            print(f"✅ Encoders: {status['encoders']}")
            print(f"✅ Velocities: {status['velocities']}")
            print(f"✅ ADC: {status['adc'][:2]}...")  # Show first 2 ADC values
        else:
            print("❌ Failed to read status")

        # Test 3: Reset encoders
        print("\n[Test 3] Resetting encoders...")
        if pr2040.reset_encoders():
            print("✅ Encoders reset")
            time.sleep(0.1)
            status = pr2040.request_status()
            if status:
                print(f"   New encoder values: {status['encoders']}")
        else:
            print("❌ Failed to reset encoders")

        print("\n" + "=" * 60)
        print("Basic communication tests completed")
        print("=" * 60)

    finally:
        pr2040.close()


def test_motor_movement():
    """Test motor movement - 10cm forward"""
    print("=" * 60)
    print("PR2040 Motor Movement Test - Forward 10cm")
    print("=" * 60)

    pr2040 = PR2040USB()

    try:
        # Parameters
        # Assuming: wheel_radius=0.033m, encoder_cpr=720
        # Wheel circumference = 2 * pi * 0.033 = 0.207m
        # Distance per count = 0.207 / 720 = 0.000288m
        # For 10cm = 0.1m: counts = 0.1 / 0.000288 = 347.2 counts

        # Velocity: 50 counts/sec (slow speed)
        # Time: 347.2 / 50 = 6.94 seconds

        target_velocity = 50.0  # counts/sec
        target_counts = 347  # ~10cm
        duration = target_counts / target_velocity  # ~6.9 seconds

        print(f"\nTarget: {target_counts} counts (~10cm)")
        print(f"Velocity: {target_velocity} counts/sec")
        print(f"Duration: {duration:.1f} seconds")

        # Reset encoders
        print("\n[1/5] Resetting encoders...")
        pr2040.reset_encoders()
        time.sleep(0.2)

        # Check initial position
        print("[2/5] Reading initial position...")
        status = pr2040.request_status()
        if status:
            initial_enc = status['encoders']
            print(f"   Initial encoders: {initial_enc}")

        # Start movement
        print(f"\n[3/5] Moving forward at {target_velocity} cps...")
        # Set same velocity for all wheels (4WD forward)
        pr2040.set_velocity_all(target_velocity, target_velocity,
                                target_velocity, target_velocity)

        # Monitor movement
        print("[4/5] Monitoring movement...")
        start_time = time.time()

        while (time.time() - start_time) < duration:
            time.sleep(0.5)
            status = pr2040.request_status()
            if status:
                elapsed = time.time() - start_time
                avg_enc = sum(status['encoders']) / 4.0
                print(f"   t={elapsed:.1f}s: avg encoder = {avg_enc:.0f} counts")

        # Stop
        print("\n[5/5] Stopping...")
        pr2040.stop_all()
        time.sleep(0.2)

        # Check final position
        status = pr2040.request_status()
        if status:
            final_enc = status['encoders']
            avg_final = sum(final_enc) / 4.0
            distance = avg_final * 0.000288  # meters

            print(f"\n📊 Results:")
            print(f"   Final encoders: {final_enc}")
            print(f"   Average: {avg_final:.0f} counts")
            print(f"   Estimated distance: {distance*100:.1f} cm")

        print("\n" + "=" * 60)
        print("Motor movement test completed")
        print("=" * 60)

    finally:
        pr2040.stop_all()
        pr2040.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='PR2040 USB Serial Test')
    parser.add_argument('--test', choices=['basic', 'move'], default='basic',
                       help='Test to run: basic (communication) or move (motor movement)')

    args = parser.parse_args()

    if args.test == 'basic':
        test_basic_communication()
    elif args.test == 'move':
        test_motor_movement()
