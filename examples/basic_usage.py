#!/usr/bin/env python3
"""
Basic example of using the motion library.
"""
import asyncio
from pykos import KOS
from motion.robot import Robot

async def main():
    # Define joint mapping for your robot
    joint_map = {
        "left_shoulder_yaw": 11,
        "left_shoulder_pitch": 12,
        "left_elbow": 13,
        "left_gripper": 14,
        # ... other joints
    }
    
    # Define joint groups (optional)
    groups = {
        "left_arm": ["left_shoulder_yaw", "left_shoulder_pitch", "left_elbow"],
        "grippers": ["left_gripper"],
    }
    
    # Create robot instance
    robot = Robot(joint_map=joint_map, groups=groups)
    
    # Connect to KOS
    async with KOS(ip="localhost", port=50051) as kos:
        # Configure robot for simulator
        await robot.configure(kos, is_real=False)
        
        # Move joints
        await robot.move(kos, {
            "left_shoulder_yaw": 45,
            "left_elbow": -30
        })
        
        # Get joint states
        states = await robot.get_states(kos)
        for name, state in states.items():
            print(f"{name}: pos={state.position:.2f}, vel={state.velocity:.2f}")
        
        # Work with a specific group
        left_arm = robot.get_group("left_arm")
        if left_arm:
            print(f"Left arm has {len(left_arm)} joints")
            
        # Zero all joints
        await robot.zero_all(kos)

if __name__ == "__main__":
    asyncio.run(main())