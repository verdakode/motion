#!/usr/bin/env python3
"""
Minimal example of using the motion library.
"""
import asyncio
from pykos import KOS
from motion.robot import Robot

async def main():
    # Create a simple robot with two joints
    robot = Robot(joint_map={"joint1": 1, "joint2": 2})
    
    # Connect to KOS
    async with KOS() as kos:
        # Configure robot
        await robot.configure(kos, is_real=False)
        
        # Move joint1 to position 45 degrees
        await robot.move(kos, {"joint1": 45})
        
        # Get joint states
        states = await robot.get_states(kos)
        print(f"Joint1 position: {states['joint1'].position}")
        
        # Zero all joints
        await robot.zero_all(kos)

if __name__ == "__main__":
    asyncio.run(main())