#!/usr/bin/env python3
"""
Test to run puppet and real robot in parallel.
"""

import asyncio
import sys
import subprocess
import time
import logging
from pykos import KOS
from motion.robot import Robot

# Configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("robot_test")


async def main():
    logger.info("Starting robot test")

    # Create two robot instances - one for sim, one for real
    sim_robot = Robot()  # Uses DEFAULT_ACTUATOR_MAPPING
    real_robot = Robot()  # Uses DEFAULT_ACTUATOR_MAPPING

    logger.info("Starting simulator server...")
    sim_process = subprocess.Popen(["kos-sim", "zbot-v2-fixed"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2)

    try:
        logger.info("Running test on simulator...")
        async with KOS(ip="localhost", port=50051) as sim_kos:
            logger.info("Connected to simulator")

            logger.info("Resetting simulator")
            await sim_kos.sim.reset()
            time.sleep(1)

            logger.info("Configuring simulator robot")
            await sim_robot.configure(sim_kos, is_real=False)

            # Start monitoring simulator robot - with quiet mode enabled to reduce console output
            await sim_robot.start_monitoring(sim_kos, interval=0.5, quiet_mode=True)

            logger.info("Connecting to real robot at 192.168.42.1...")
            async with KOS(ip="192.168.42.1") as real_kos:
                logger.info("Connected to real robot")

                logger.info("Configuring real robot")
                await real_robot.configure(real_kos, is_real=True)

                # Start monitoring real robot - with quiet mode enabled to reduce console output
                await real_robot.start_monitoring(real_kos, interval=0.5, quiet_mode=True)

                # Get initial states for both robots
                logger.info("[SIM] Getting initial actuator states")
                sim_states = await sim_robot.get_states(sim_kos)
                for name, state in sim_states.items():
                    logger.info(
                        f"[SIM] Initial state of {name}: position={state.position:.2f}, velocity={state.velocity:.2f}"
                    )

                logger.info("[REAL] Getting initial actuator states")
                real_states = await real_robot.get_states(real_kos)
                for name, state in real_states.items():
                    logger.info(
                        f"[REAL] Initial state of {name}: position={state.position:.2f}, velocity={state.velocity:.2f}"
                    )

                # Zero all joints (non-blocking)
                logger.info("Zeroing all joints with velocity 60...")
                await sim_robot.zero_all(sim_kos, velocity=60)
                await real_robot.zero_all(real_kos, velocity=60)
                # Just a short sleep to let commands send
                await asyncio.sleep(0.5)

                # Test each joint
                for joint_name in sim_robot.joints.keys():
                    logger.info(f"Testing {joint_name}")
                    test_angle = -45
                    # Adjust angle for certain joints that need reversed direction
                    if joint_name in ["left_shoulder_pitch", "right_shoulder_yaw", "right_elbow", "right_gripper"]:
                        adjusted_angle = -test_angle
                    else:
                        adjusted_angle = test_angle

                    # Move joint one at a time - simulator first, then real robot
                    logger.info(f"Moving {joint_name} to {adjusted_angle} degrees")
                    await sim_robot.move(sim_kos, {joint_name: adjusted_angle}, velocity=80)
                    await asyncio.sleep(0.1)  # Small wait between commands
                    await real_robot.move(real_kos, {joint_name: adjusted_angle}, velocity=80)
                    
                    # Minimal wait to let command complete
                    await asyncio.sleep(0.2)

                    # Check final position on both robots
                    sim_states = await sim_robot.get_states(sim_kos, [joint_name])
                    real_states = await real_robot.get_states(real_kos, [joint_name])

                    sim_pos = sim_states[joint_name].position
                    real_pos = real_states[joint_name].position

                    logger.info(f"[SIM] {joint_name}: position={sim_pos:.2f} (target was {adjusted_angle})")
                    logger.info(f"[REAL] {joint_name}: position={real_pos:.2f} (target was {adjusted_angle})")

                    # Return to zero one at a time
                    logger.info(f"Returning {joint_name} to zero")
                    await sim_robot.move(sim_kos, {joint_name: 0}, velocity=80)
                    await asyncio.sleep(0.1)  # Small wait between commands
                    await real_robot.move(real_kos, {joint_name: 0}, velocity=80)
                    await asyncio.sleep(0.2)

                # Stop monitoring both robots
                await sim_robot.stop_monitoring()
                await real_robot.stop_monitoring()

    except Exception as e:
        logger.error(f"Error during test: {e}", exc_info=True)
    finally:
        # Stop monitoring if still running
        if sim_robot.is_monitoring:
            await sim_robot.stop_monitoring()
        if real_robot.is_monitoring:
            await real_robot.stop_monitoring()

        logger.info("Terminating simulator")
        sim_process.terminate()
        sim_process.wait()
        logger.info("Test completed")


if __name__ == "__main__":
    asyncio.run(main())