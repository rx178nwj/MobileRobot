#!/usr/bin/env python3
"""
Test USB Serial-Based Edge Services

Tests motor control and odometry via USB serial connection.
"""

import asyncio
import json
import time
import sys
import os

# Add edge_services to path
sys.path.insert(0, '/home/pi/MobileRobot/edge_services')

from hardware.pr2040_usb_driver import PR2040USBDriver
from motor_service import MotorControlService
from odometry_service import OdometryService


def test_driver_basic():
    """Test basic PR2040 USB driver functionality"""
    print("=" * 60)
    print("Test 1: Basic USB Driver Functionality")
    print("=" * 60)

    driver = PR2040USBDriver(port='/dev/ttyACM0')

    if not driver.connected:
        print("❌ Failed to connect to PR2040")
        return False

    print("✅ Connected to PR2040")

    # Test encoder reading
    encoders = driver.read_all_encoders()
    print(f"✅ Encoder readings: {encoders}")

    # Test temperature reading
    temp = driver.read_temperature()
    print(f"✅ Temperature: {temp:.2f}°C")

    # Reset encoders
    driver.reset_encoders()
    time.sleep(0.2)
    encoders = driver.read_all_encoders()
    print(f"✅ Encoders after reset: {encoders}")

    driver.close()
    print("\n✅ Test 1 PASSED\n")
    return True


def test_motor_service():
    """Test motor control service"""
    print("=" * 60)
    print("Test 2: Motor Control Service")
    print("=" * 60)

    config = {
        'wheel_base': 0.16,
        'wheel_radius': 0.033,
        'encoder_cpr': 827.2,
        'max_linear_velocity': 0.5,
        'max_angular_velocity': 2.0,
        'cmd_vel_timeout': 0.5,
        'host': '0.0.0.0',
        'port': 8001,
        'usb_port': '/dev/ttyACM0'
    }

    service = MotorControlService(config)

    if not service.driver.connected:
        print("❌ Failed to connect to PR2040")
        service.shutdown()
        return False

    print("✅ Motor service initialized")

    # Test velocity command
    print("\n[Testing] Setting velocity: 0.1 m/s forward")
    service.set_motor_velocities(0.1, 0.0)
    time.sleep(1.0)

    # Stop
    print("[Testing] Stopping motors")
    service.stop_motors()

    service.shutdown()
    print("\n✅ Test 2 PASSED\n")
    return True


def test_odometry_service():
    """Test odometry service"""
    print("=" * 60)
    print("Test 3: Odometry Service")
    print("=" * 60)

    config = {
        'wheel_base': 0.16,
        'wheel_radius': 0.033,
        'encoder_cpr': 827.2,
        'publish_rate': 50.0,
        'host': '0.0.0.0',
        'port': 8002,
        'usb_port': '/dev/ttyACM0'
    }

    service = OdometryService(config)

    if not service.driver.connected:
        print("❌ Failed to connect to PR2040")
        service.shutdown()
        return False

    print("✅ Odometry service initialized")

    # Reset odometry
    service.reset_odometry()
    service.driver.reset_encoders()
    time.sleep(0.2)

    # Read encoders
    encoders = service.driver.read_all_encoders()
    print(f"✅ Initial encoders: {encoders}")

    # Calculate odometry
    odom_msg = service.calculate_odometry(encoders)
    print(f"✅ Odometry calculated:")
    print(f"   Position: x={odom_msg['pose']['x']:.3f}, y={odom_msg['pose']['y']:.3f}, theta={odom_msg['pose']['theta']:.3f}")
    print(f"   Velocity: linear={odom_msg['twist']['linear']:.3f}, angular={odom_msg['twist']['angular']:.3f}")

    service.shutdown()
    print("\n✅ Test 3 PASSED\n")
    return True


def test_motor_movement():
    """Test actual motor movement"""
    print("=" * 60)
    print("Test 4: Motor Movement (5cm forward)")
    print("=" * 60)

    driver = PR2040USBDriver(port='/dev/ttyACM0')

    if not driver.connected:
        print("❌ Failed to connect to PR2040")
        return False

    # Reset encoders
    driver.reset_encoders()
    time.sleep(0.3)

    # Read initial
    initial_encoders = driver.read_all_encoders()
    print(f"Initial encoders: {initial_encoders}")

    # Move forward at 100 cps for 2 seconds
    print("\nMoving forward at 100 counts/sec...")
    target_velocity = 100.0
    duration = 2.0

    driver.set_wheel_velocities((target_velocity, target_velocity,
                                 target_velocity, target_velocity))

    # Monitor
    start_time = time.time()
    while (time.time() - start_time) < duration:
        encoders = driver.read_all_encoders()
        elapsed = time.time() - start_time
        avg = sum(encoders) / 4.0
        print(f"   t={elapsed:.1f}s: {encoders}, avg={avg:.0f}")
        time.sleep(0.5)

    # Stop
    driver.stop_all()
    time.sleep(0.3)

    # Final
    final_encoders = driver.read_all_encoders()
    avg_counts = sum(final_encoders) / 4.0

    # Calculate distance (using calibrated CPR)
    WHEEL_CIRCUMFERENCE = 212.06  # mm
    ENCODER_CPR = 827.2
    COUNTS_TO_MM = WHEEL_CIRCUMFERENCE / ENCODER_CPR
    distance_mm = avg_counts * COUNTS_TO_MM

    print(f"\nFinal encoders: {final_encoders}")
    print(f"Average counts: {avg_counts:.0f}")
    print(f"Distance traveled: {distance_mm:.1f} mm ({distance_mm/10:.1f} cm)")

    driver.close()

    if avg_counts > 50:
        print("\n✅ Test 4 PASSED (motors are moving)\n")
        return True
    else:
        print("\n❌ Test 4 FAILED (motors not moving enough)\n")
        return False


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("USB Serial-Based Edge Services Test Suite")
    print("=" * 60 + "\n")

    results = []

    # Test 1: Basic driver
    try:
        results.append(("Basic USB Driver", test_driver_basic()))
    except Exception as e:
        print(f"❌ Test 1 FAILED with exception: {e}\n")
        results.append(("Basic USB Driver", False))

    # Test 2: Motor service
    try:
        results.append(("Motor Service", test_motor_service()))
    except Exception as e:
        print(f"❌ Test 2 FAILED with exception: {e}\n")
        results.append(("Motor Service", False))

    # Test 3: Odometry service
    try:
        results.append(("Odometry Service", test_odometry_service()))
    except Exception as e:
        print(f"❌ Test 3 FAILED with exception: {e}\n")
        results.append(("Odometry Service", False))

    # Test 4: Motor movement
    try:
        results.append(("Motor Movement", test_motor_movement()))
    except Exception as e:
        print(f"❌ Test 4 FAILED with exception: {e}\n")
        results.append(("Motor Movement", False))

    # Summary
    print("=" * 60)
    print("Test Summary")
    print("=" * 60)

    for test_name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{test_name:.<40} {status}")

    print("=" * 60)

    all_passed = all(result[1] for result in results)
    if all_passed:
        print("\n🎉 All tests PASSED!")
    else:
        print("\n⚠️  Some tests FAILED")

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
