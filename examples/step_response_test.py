#!/usr/bin/env python3
"""Test step response characteristics of joints."""

import asyncio
import time
import json
import os
from datetime import datetime
import argparse
import matplotlib.pyplot as plt
from pykos import KOS
from motion.robot import Robot, RobotConfig


async def main():
    parser = argparse.ArgumentParser(description="Test joint step response")
    parser.add_argument(
        "--joint", type=str, default="right_elbow", help="Joint to test"
    )
    parser.add_argument(
        "--settle", type=float, default=5.0, help="Time to wait for settling (seconds)"
    )
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Define step sequence: (position, duration)
    steps = [
        (0.0, 1.0),  # Start at zero
        (20.0, 1.0),  # Step up to 20° (0.5Hz)
        (-20.0, 1.0),  # Step down to -20°
        (20.0, 1.0),  # Step up to 20°
        (-20.0, 1.0),  # Step down to -20°
        (20.0, 1.0),  # Step up to 20°
        (-20.0, 1.0),  # Step down to -20°
        (0.0, 1.0),  # Return to zero
    ]

    # Data recording
    sim_data = {"time": [], "position": [], "target": [], "velocity": []}
    real_data = {"time": [], "position": [], "target": [], "velocity": []}

    config = RobotConfig(
        sim_gains=(200, 32),
        real_gains=(32, 50),
        max_torque=150.0,
    )

    sim_robot = Robot(config=config)
    real_robot = Robot(config=config)

    try:
        print("Connecting to simulator...")
        async with KOS(ip="localhost") as sim_kos:
            await sim_robot.configure(sim_kos, is_real=False)

            print("Connecting to real robot...")
            async with KOS(ip="192.168.42.1") as real_kos:
                await real_robot.configure(real_kos, is_real=True)

                # Zero both robots
                print("Zeroing joints...")
                await sim_robot.zero_all(sim_kos)
                await real_robot.zero_all(real_kos)
                await asyncio.sleep(2.0)

                # Run through step sequence
                print("\nRunning step sequence...")
                overall_start = time.time()

                for target_pos, duration in steps:
                    print(f"\nMoving to {target_pos}° for {duration}s")

                    # Command both robots simultaneously
                    await sim_robot.move(sim_kos, {args.joint: target_pos})
                    await real_robot.move(real_kos, {args.joint: target_pos})

                    start_time = time.time()
                    while time.time() - start_time < duration:
                        t = time.time() - overall_start

                        # Get simulator state
                        sim_states = await sim_robot.get_states(sim_kos, [args.joint])
                        sim_pos = sim_states[args.joint].position
                        sim_vel = sim_states[args.joint].velocity

                        # Get real robot state
                        real_states = await real_robot.get_states(
                            real_kos, [args.joint]
                        )
                        real_pos = real_states[args.joint].position
                        real_vel = real_states[args.joint].velocity

                        # Record data
                        sim_data["time"].append(t)
                        sim_data["position"].append(sim_pos)
                        sim_data["target"].append(target_pos)
                        sim_data["velocity"].append(sim_vel)

                        real_data["time"].append(t)
                        real_data["position"].append(real_pos)
                        real_data["target"].append(target_pos)
                        real_data["velocity"].append(real_vel)

                        print(
                            f"Time: {t:.3f}s | Sim: {sim_pos:.2f}° | Real: {real_pos:.2f}°"
                        )

                # Return to zero
                print("\nReturning to zero...")
                await sim_robot.zero_all(sim_kos)
                await real_robot.zero_all(real_kos)

    finally:
        # Save data
        os.makedirs("plots", exist_ok=True)
        with open(f"plots/step_test_sim_{timestamp}.json", "w") as f:
            json.dump(sim_data, f)
        with open(f"plots/step_test_real_{timestamp}.json", "w") as f:
            json.dump(real_data, f)

        # Create plots
        plt.figure(figsize=(12, 8))

        # Position subplot
        plt.subplot(2, 1, 1)
        plt.plot(sim_data["time"], sim_data["target"], "k--", label="Target")
        plt.plot(sim_data["time"], sim_data["position"], "b-", label="Simulator")
        plt.plot(real_data["time"], real_data["position"], "r-", label="Real Robot")
        plt.title(f"Step Response - {args.joint}")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (degrees)")
        plt.legend()
        plt.grid(True)

        # Velocity subplot
        plt.subplot(2, 1, 2)
        plt.plot(sim_data["time"], sim_data["velocity"], "b-", label="Simulator")
        plt.plot(real_data["time"], real_data["velocity"], "r-", label="Real Robot")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (degrees/s)")
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig(f"plots/step_test_{timestamp}.png")


if __name__ == "__main__":
    asyncio.run(main())
