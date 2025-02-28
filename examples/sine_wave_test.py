#!/usr/bin/env python3
"""
Sine wave test for right_ankle that runs both simulator and real robot simultaneously and compares results.
"""

import asyncio
import math
import time
import json
import subprocess
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime
from pykos import KOS
from motion.robot import Robot, RobotConfig


async def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Run sine wave test on right ankle")
    parser.add_argument(
        "--freq", type=float, default=1.0, help="Frequency of sine wave in Hz"
    )
    parser.add_argument(
        "--amp", type=float, default=10.0, help="Amplitude of sine wave in degrees"
    )
    parser.add_argument(
        "--duration", type=float, default=10.0, help="Duration of test in seconds"
    )
    args = parser.parse_args()

    # Create timestamp for data files
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Custom configuration with specified gains
    config = RobotConfig(
        sim_gains=(200, 50),  # More conservative gains
        real_gains=(32, 32),  # These are working well for real robot
        max_torque=100.0,
    )

    # Create two robot instances
    sim_robot = Robot(config=config)
    real_robot = Robot(config=config)

    # Only testing right_ankle
    joint_to_test = "right_ankle"

    # Start simulator server
    print("Starting simulator server...")

    time.sleep(2)

    # Recording data
    sim_data = {"time": [], "position": [], "expected": []}
    real_data = {"time": [], "position": [], "expected": []}

    try:
        # Connect to simulator
        print("Connecting to simulator...")
        async with KOS(ip="localhost") as sim_kos:
            # Configure simulator robot
            await sim_robot.configure(sim_kos, is_real=False)

            # Reset simulator and configure actuators
            print("Resetting simulator...")
            await sim_kos.sim.reset()
            # Reset all actuator states
            await sim_kos.actuator.command_actuators(
                [
                    {
                        "actuator_id": joint.actuator_id,
                        "position": 0.0,
                        "velocity": 10.0,  # Always need velocity
                        "torque": 100.0,  # Always need torque
                    }
                    for joint in sim_robot.joints.values()
                ]
            )
            await asyncio.sleep(1.0)

            print("Configuring simulator actuators...")
            for joint_name, joint in sim_robot.joints.items():
                await sim_kos.actuator.configure_actuator(
                    actuator_id=joint.actuator_id,
                    kp=config.sim_gains[0],
                    kd=config.sim_gains[1],
                    max_torque=config.max_torque,
                )
            await asyncio.sleep(1.0)

            # Zero all joints in simulator
            print("Zeroing simulator joints...")
            await sim_robot.zero_all(sim_kos)
            await asyncio.sleep(5.0)  # Give simulator more time to settle

            # Connect to real robot
            print("Connecting to real robot...")
            async with KOS(ip="192.168.42.1") as real_kos:
                # Configure real robot
                await real_robot.configure(real_kos, is_real=True)

                # Zero all joints in real robot
                print("Zeroing real robot joints...")
                await real_robot.zero_all(real_kos)
                await asyncio.sleep(2.0)

                # Parameters
                safe_position = 20.0  # Starting at 20 degrees
                amplitude = args.amp
                frequency = args.freq
                duration = args.duration

                # Move to starting position
                print(f"Moving joint to safe position {safe_position}°")
                # Try up to 3 times to reach starting position for simulator
                for attempt in range(3):
                    # Move in smaller steps
                    sim_states = await sim_robot.get_states(sim_kos, [joint_to_test])
                    current_pos = sim_states[joint_to_test].position
                    steps = 5
                    for step in range(steps):
                        target = (
                            current_pos
                            + (safe_position - current_pos) * (step + 1) / steps
                        )
                        await sim_robot.move(
                            sim_kos,
                            {joint_to_test: target},
                            velocity=10.0,
                        )
                        await asyncio.sleep(0.5)
                    await asyncio.sleep(2.0)
                    sim_states = await sim_robot.get_states(sim_kos, [joint_to_test])
                    if abs(sim_states[joint_to_test].position - safe_position) < 3.0:
                        print(
                            f"Simulator reached starting position: {sim_states[joint_to_test].position:.2f}°"
                        )
                        break
                    print(
                        f"Attempt {attempt+1}: Position {sim_states[joint_to_test].position:.2f}° not close enough to {safe_position}°"
                    )
                    if abs(sim_states[joint_to_test].position - safe_position) >= 3.0:
                        print(
                            "WARNING: Simulator did not reach starting position, continuing anyway"
                        )

                # Now do the same for real robot
                print("Moving real robot to starting position...")
                for attempt in range(3):
                    real_states = await real_robot.get_states(real_kos, [joint_to_test])
                    current_pos = real_states[joint_to_test].position
                    steps = 5
                    for step in range(steps):
                        target = (
                            current_pos
                            + (safe_position - current_pos) * (step + 1) / steps
                        )
                        await real_robot.move(
                            real_kos,
                            {joint_to_test: target},
                            velocity=10.0,
                        )
                        await asyncio.sleep(0.5)
                    await asyncio.sleep(2.0)
                    real_states = await real_robot.get_states(real_kos, [joint_to_test])
                    if (
                        abs(real_states[joint_to_test].position - safe_position) < 1.0
                    ):  # Tighter tolerance for real robot
                        print(
                            f"Real robot reached starting position: {real_states[joint_to_test].position:.2f}°"
                        )
                        break
                    print(
                        f"Attempt {attempt+1}: Position {real_states[joint_to_test].position:.2f}° not close enough to {safe_position}°"
                    )
                    if abs(real_states[joint_to_test].position - safe_position) >= 1.0:
                        print(
                            "WARNING: Real robot did not reach starting position, continuing anyway"
                        )
                else:
                    raise Exception(
                        "Failed to reach starting position after 3 attempts"
                    )

                # Verify both robots are at starting position
                sim_states = await sim_robot.get_states(sim_kos, [joint_to_test])
                real_states = await real_robot.get_states(real_kos, [joint_to_test])
                print(
                    f"Starting positions - Sim: {sim_states[joint_to_test].position:.2f}°, Real: {real_states[joint_to_test].position:.2f}°"
                )

                # Pre-calculate expected positions for the entire duration
                expected_positions = []
                sample_times = []
                t = 0
                while t < duration:
                    expected_pos = safe_position + amplitude * math.sin(
                        2 * math.pi * frequency * t
                    )
                    expected_positions.append(expected_pos)
                    sample_times.append(t)
                    t += 0.05

                # Run sine wave
                print(f"Starting sine wave (±{amplitude}° at {frequency} Hz)")

                # After zeroing both robots, run tests one at a time

                print("\nRunning simulator test...")
                try:
                    print("Letting simulator stabilize...")
                    await asyncio.sleep(1.0)  # Additional stabilization time
                    print("Starting sine wave motion...")
                    # Run simulator test
                    start_time = time.time()
                    last_time = start_time
                    while time.time() - start_time < duration:
                        t = time.time() - start_time
                        dt = time.time() - last_time  # Get actual timestep
                        last_time = time.time()

                        # Find closest expected position
                        idx = min(int(t / 0.05), len(expected_positions) - 1)
                        expected_pos = expected_positions[idx]

                        # Calculate velocity from next position
                        next_idx = min(idx + 1, len(expected_positions) - 1)
                        velocity = (expected_positions[next_idx] - expected_pos) / dt

                        # Send command to simulator
                        await sim_robot.move(
                            sim_kos,
                            {joint_to_test: expected_pos},
                            velocity=velocity,
                        )
                        # Get state immediately after command
                        sim_states = await sim_robot.get_states(
                            sim_kos, [joint_to_test]
                        )

                        # Record data
                        sim_data["time"].append(t)
                        sim_data["position"].append(sim_states[joint_to_test].position)
                        sim_data["expected"].append(expected_pos)

                        print(
                            f"Time: {t:.2f}s | Expected: {expected_pos:.2f}° | Sim: {sim_states[joint_to_test].position:.2f}°"
                        )
                except Exception as e:
                    print(f"Error during simulator test: {e}")
                finally:
                    # Return simulator to safe position
                    await sim_robot.move(sim_kos, {joint_to_test: safe_position})
                    await asyncio.sleep(2.0)

                print("\nRunning real robot test...")
                try:
                    # Run real robot test
                    start_time = time.time()
                    while time.time() - start_time < duration:
                        t = time.time() - start_time
                        # Find closest expected position
                        idx = min(int(t / 0.05), len(expected_positions) - 1)
                        expected_pos = expected_positions[idx]

                        # Calculate velocity from next position
                        next_idx = min(idx + 1, len(expected_positions) - 1)
                        velocity = (expected_positions[next_idx] - expected_pos) / 0.05

                        # Send command to real robot only
                        await real_robot.move(
                            real_kos,
                            {joint_to_test: expected_pos},
                            velocity=velocity,
                        )

                        # Get real robot position
                        real_states = await real_robot.get_states(
                            real_kos, [joint_to_test]
                        )

                        # Record data
                        real_data["time"].append(t)
                        real_data["position"].append(
                            real_states[joint_to_test].position
                        )
                        real_data["expected"].append(expected_pos)

                        print(
                            f"Time: {t:.2f}s | Expected: {expected_pos:.2f}° | Real: {real_states[joint_to_test].position:.2f}°"
                        )
                        await asyncio.sleep(0.05)
                except Exception as e:
                    print(f"Error during real robot test: {e}")
                finally:
                    # Return real robot to safe position
                    print(f"Returning to {safe_position}°")
                    await real_robot.move(real_kos, {joint_to_test: safe_position})
                    await asyncio.sleep(2.0)

    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        # Save data
        os.makedirs("plots", exist_ok=True)
        with open(f"plots/sine_test_sim_{timestamp}.json", "w") as f:
            json.dump(sim_data, f)
        with open(f"plots/sine_test_real_{timestamp}.json", "w") as f:
            json.dump(real_data, f)

        # Create comparison plot with side-by-side layout
        plt.figure(figsize=(12, 6))

        # Simulator subplot (left)
        plt.subplot(1, 2, 1)
        plt.plot(sim_data["time"], sim_data["expected"], "b--", label="Expected")
        plt.plot(sim_data["time"], sim_data["position"], "r-", label="Actual")
        plt.title(f"Simulator - {joint_to_test}")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (degrees)")
        plt.legend()
        plt.grid(True)

        # Real robot subplot (right)
        plt.subplot(1, 2, 2)
        plt.plot(real_data["time"], real_data["expected"], "b--", label="Expected")
        plt.plot(real_data["time"], real_data["position"], "r-", label="Actual")
        plt.title(f"Real Robot - {joint_to_test}")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (degrees)")
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig(f"plots/sine_test_comparison_{timestamp}.png")

        print("Test completed and plots generated")


if __name__ == "__main__":
    asyncio.run(main())
