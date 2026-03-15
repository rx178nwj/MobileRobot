#!/usr/bin/env python3
"""
PR2040 Motor Test - Complete Implementation
Based on CommandReference.md
"""

import serial
import struct
import time
import sys


class PR2040USB:
    """PR2040 Motor Driver USB Serial Interface"""

    PACKET_HEADER = 0xAA

    # Commands (from CommandReference.md)
    CMD_SET_MOTORS = 0x01
    CMD_STOP_ALL = 0x02
    CMD_BRAKE_ALL = 0x03
    CMD_SET_MOTOR_SINGLE = 0x04
    CMD_REQUEST_STATUS = 0x10
    CMD_RESET_ENCODERS = 0x11
    CMD_SET_MODE_ALL = 0x20
    CMD_SET_VEL_ALL = 0x22

    # Responses
    RESP_ACK = 0x80
    RESP_NAK = 0x81
    RESP_STATUS = 0x91

    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"✅ Connected to {port}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop_all()
            self.ser.close()

    def _calc_checksum(self, cmd, length, data):
        cs = cmd ^ length
        for b in data:
            cs ^= b
        return cs

    def _send_cmd(self, cmd, data=b''):
        length = len(data)
        checksum = self._calc_checksum(cmd, length, data)
        packet = bytes([self.PACKET_HEADER, cmd, length]) + data + bytes([checksum])
        self.ser.write(packet)

    def _drain_status_packets(self, duration=0.2):
        """Drain automatic status packets (100Hz)"""
        time.sleep(duration)
        self.ser.reset_input_buffer()

    def stop_all(self):
        """Stop all motors (STOP_ALL also sets DIRECT mode)"""
        self._send_cmd(self.CMD_STOP_ALL)
        self._drain_status_packets()

    def reset_encoders(self):
        """Reset all encoder counts"""
        self._send_cmd(self.CMD_RESET_ENCODERS)
        self._drain_status_packets()

    def set_mode_all(self, mode):
        """Set control mode for all motors (0=DIRECT, 1=VELOCITY, 2=POSITION)"""
        self._send_cmd(self.CMD_SET_MODE_ALL, bytes([mode, mode, mode, mode]))
        self._drain_status_packets()

    def set_velocity_all(self, v0, v1, v2, v3):
        """Set velocity targets [counts/sec] for all motors"""
        data = struct.pack('<ffff', float(v0), float(v1), float(v2), float(v3))
        self._send_cmd(self.CMD_SET_VEL_ALL, data)
        self._drain_status_packets(0.05)

    def set_motors_duty(self, d0, d1, d2, d3):
        """Set duty cycle [-1000..+1000] for all motors (DIRECT mode)"""
        data = struct.pack('<hhhh', int(d0), int(d1), int(d2), int(d3))
        self._send_cmd(self.CMD_SET_MOTORS, data)
        self._drain_status_packets(0.05)

    def request_status(self):
        """Request status and read latest RESP_STATUS"""
        # Drain old packets
        self.ser.reset_input_buffer()

        # Request new status
        self._send_cmd(self.CMD_REQUEST_STATUS)

        # Wait for response (100Hz auto-send, so multiple may arrive)
        time.sleep(0.05)

        # Read all available data
        if self.ser.in_waiting < 48:  # Header(1) + Type(1) + Len(1) + Data(44) + CS(1)
            return None

        # Find and parse the most recent STATUS packet
        latest_status = None
        while self.ser.in_waiting >= 48:
            # Look for header
            b = self.ser.read(1)
            if len(b) == 0 or b[0] != self.PACKET_HEADER:
                continue

            # Read type and length
            type_byte = self.ser.read(1)
            if len(type_byte) == 0:
                continue

            type_val = type_byte[0]
            length = self.ser.read(1)[0]

            # Check if it's STATUS
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

                    latest_status = {
                        'encoders': encoders,
                        'velocities': velocities,
                        'adc': adc,
                        'timestamp': timestamp
                    }
            else:
                # Skip this packet
                if length > 0:
                    self.ser.read(length + 1)  # data + checksum

        return latest_status


def test_all_motors_direct():
    """Test all 4 motors in DIRECT mode"""
    print("=" * 70)
    print("PR2040 Motor Test - DIRECT Mode (Individual Motor Test)")
    print("=" * 70)

    pr2040 = PR2040USB()

    try:
        # Set DIRECT mode
        print("\n[1] Setting DIRECT mode...")
        pr2040.set_mode_all(0)  # 0 = DIRECT

        # Reset encoders
        print("[2] Resetting encoders...")
        pr2040.reset_encoders()
        time.sleep(0.3)

        # Test each motor
        duty = 400  # 40% duty
        duration = 2.0

        for motor_idx in range(4):
            print(f"\n[{motor_idx+3}] Testing Motor {motor_idx} at duty={duty}...")

            # Set duty for this motor only
            duties = [0, 0, 0, 0]
            duties[motor_idx] = duty
            pr2040.set_motors_duty(*duties)

            # Monitor
            start_time = time.time()
            initial_enc = None

            while (time.time() - start_time) < duration:
                status = pr2040.request_status()
                if status:
                    if initial_enc is None:
                        initial_enc = status['encoders'][motor_idx]

                    enc = status['encoders'][motor_idx]
                    delta = enc - initial_enc
                    elapsed = time.time() - start_time

                    if elapsed >= 0.5:  # Print every 0.5s
                        print(f"   t={elapsed:.1f}s: enc[{motor_idx}]={enc} (Δ={delta})")

                time.sleep(0.5)

            # Stop
            pr2040.stop_all()
            time.sleep(0.2)

            # Final
            status = pr2040.request_status()
            if status:
                final_enc = status['encoders'][motor_idx]
                total_delta = final_enc - initial_enc if initial_enc else final_enc

                if abs(total_delta) > 10:
                    print(f"   ✅ Motor {motor_idx}: WORKING ({total_delta} counts)")
                else:
                    print(f"   ❌ Motor {motor_idx}: NOT MOVING ({total_delta} counts)")

        print("\n" + "=" * 70)

    finally:
        pr2040.close()


def test_move_10cm_velocity():
    """Test: Move forward 10cm using VELOCITY mode"""
    print("=" * 70)
    print("PR2040 Motor Test - VELOCITY Mode (10cm Forward)")
    print("=" * 70)

    pr2040 = PR2040USB()

    try:
        # Parameters (from CommandReference.md)
        WHEEL_CIRCUMFERENCE = 212.06  # mm
        ENCODER_CPR = 827.2  # counts/revolution (calibrated value)
        COUNTS_TO_MM = 0.2564  # conversion factor

        target_distance_mm = 100  # 10cm
        target_counts = int(target_distance_mm / COUNTS_TO_MM)  # ≈ 390 counts
        target_velocity = 200.0  # counts/sec (moderate speed)
        duration = target_counts / target_velocity  # ≈ 2 seconds

        print(f"\nTarget: {target_distance_mm}mm = {target_counts} counts")
        print(f"Velocity: {target_velocity} counts/sec")
        print(f"Duration: {duration:.1f} seconds\n")

        # Set VELOCITY mode
        print("[1/6] Setting VELOCITY mode...")
        pr2040.set_mode_all(1)  # 1 = VELOCITY
        time.sleep(0.2)

        # Reset encoders
        print("[2/6] Resetting encoders...")
        pr2040.reset_encoders()
        time.sleep(0.3)

        # Check initial
        print("[3/6] Reading initial position...")
        status = pr2040.request_status()
        if status:
            print(f"   Initial encoders: {status['encoders']}")
            print(f"   Initial velocities: {status['velocities']}")

        # Start movement
        print(f"\n[4/6] Starting forward movement at {target_velocity} cps...")
        pr2040.set_velocity_all(target_velocity, target_velocity,
                               target_velocity, target_velocity)

        # Monitor
        print("[5/6] Monitoring movement...")
        start_time = time.time()

        while (time.time() - start_time) < duration:
            status = pr2040.request_status()
            if status:
                elapsed = time.time() - start_time
                avg_enc = sum(status['encoders']) / 4.0
                avg_vel = sum(status['velocities']) / 4.0
                distance_mm = avg_enc * COUNTS_TO_MM

                print(f"   t={elapsed:.1f}s: enc={status['encoders']}, avg={avg_enc:.0f}, vel={avg_vel:.0f}cps, dist={distance_mm:.1f}mm")

            time.sleep(0.5)

        # Stop
        print("\n[6/6] Stopping...")
        pr2040.stop_all()
        time.sleep(0.3)

        # Final result
        status = pr2040.request_status()
        if status:
            avg_final = sum(status['encoders']) / 4.0
            distance_mm = avg_final * COUNTS_TO_MM

            print(f"\n📊 Final Results:")
            print(f"   Encoders: {status['encoders']}")
            print(f"   Average: {avg_final:.0f} counts")
            print(f"   Distance: {distance_mm:.1f} mm ({distance_mm/10:.1f} cm)")

            # Check working motors
            working = [i for i, enc in enumerate(status['encoders']) if abs(enc) > 10]
            not_working = [i for i in range(4) if i not in working]

            if working:
                print(f"   ✅ Working motors: {working}")
            if not_working:
                print(f"   ❌ Not moving: {not_working}")

        print("\n" + "=" * 70)

    finally:
        pr2040.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='PR2040 Motor Test')
    parser.add_argument('--test', choices=['direct', 'velocity', 'move'],
                       default='move',
                       help='Test mode: direct (test each motor), velocity (10cm forward), move (same as velocity)')
    args = parser.parse_args()

    if args.test == 'direct':
        test_all_motors_direct()
    elif args.test in ['velocity', 'move']:
        test_move_10cm_velocity()
