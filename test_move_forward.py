#!/usr/bin/env python3
"""
Test script: Move robot forward 10cm

This script sends velocity commands directly to the motor service via WebSocket.
"""

import asyncio
import websockets
import json
import time


async def move_forward_10cm():
    """Move robot forward 10cm"""
    motor_uri = "ws://localhost:8001"

    # Parameters
    target_distance = 0.10  # 10cm in meters
    velocity = 0.1  # 0.1 m/s
    duration = target_distance / velocity  # 1.0 second

    print(f"Connecting to motor service at {motor_uri}...")

    try:
        async with websockets.connect(motor_uri) as websocket:
            print("✅ Connected to motor service")

            # Send forward command
            cmd = {
                "type": "cmd_vel",
                "linear": velocity,
                "angular": 0.0
            }

            print(f"📤 Sending forward command: {velocity} m/s for {duration} seconds")
            await websocket.send(json.dumps(cmd))

            # Wait for response
            response = await websocket.recv()
            resp_data = json.loads(response)
            print(f"📥 Motor service response: {resp_data}")

            # Wait for duration
            print(f"⏱️  Moving forward for {duration} seconds...")
            await asyncio.sleep(duration)

            # Send stop command
            stop_cmd = {
                "type": "cmd_vel",
                "linear": 0.0,
                "angular": 0.0
            }

            print("🛑 Sending stop command")
            await websocket.send(json.dumps(stop_cmd))

            # Wait for response
            response = await websocket.recv()
            resp_data = json.loads(response)
            print(f"📥 Motor service response: {resp_data}")

            print(f"✅ Successfully moved forward ~{target_distance*100:.0f}cm")

    except Exception as e:
        print(f"❌ Error: {e}")
        return False

    return True


async def check_odometry():
    """Check current odometry"""
    odom_uri = "ws://localhost:8002"

    print(f"\nConnecting to odometry service at {odom_uri}...")

    try:
        async with websockets.connect(odom_uri) as websocket:
            print("✅ Connected to odometry service")

            # Receive odometry message
            message = await websocket.recv()
            data = json.loads(message)

            print("\n📊 Current Odometry:")
            print(f"  Position: x={data['pose']['x']:.4f}m, y={data['pose']['y']:.4f}m, theta={data['pose']['theta']:.4f}rad")
            print(f"  Velocity: linear={data['twist']['linear']:.4f}m/s, angular={data['twist']['angular']:.4f}rad/s")
            print(f"  Encoders: {data['encoders']}")

    except Exception as e:
        print(f"❌ Error reading odometry: {e}")


async def main():
    """Main function"""
    print("=" * 60)
    print("Mobile Robot - Move Forward 10cm Test")
    print("=" * 60)

    # Check initial odometry
    print("\n[1/3] Checking initial odometry...")
    await check_odometry()

    # Move forward
    print("\n[2/3] Moving forward 10cm...")
    success = await move_forward_10cm()

    if success:
        # Wait a bit for odometry to update
        await asyncio.sleep(0.5)

        # Check final odometry
        print("\n[3/3] Checking final odometry...")
        await check_odometry()

    print("\n" + "=" * 60)
    print("Test completed!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
