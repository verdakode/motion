#!/usr/bin/env python3
"""
Example of using the monitoring functionality.
"""
import asyncio
import time
from pykos import KOS
from motion.robot import Robot, RobotConfig

async def main():
    # Create custom configuration with monitoring enabled
    config = RobotConfig(
        monitor_interval=0.2,  # Poll every 200ms
        log_to_file=True,      # Save data to CSV
        log_directory="./robot_logs"  # Directory for log files
    )
    
    # Create robot with monitoring config
    robot = Robot(config=config)
    
    # Connect to KOS
    async with KOS() as kos:
        # Configure robot for simulator
        await robot.configure(kos, is_real=False)
        
        # Start monitoring in background
        await robot.start_monitoring(kos)
        
        # Move left arm joints
        await robot.move(kos, {
            "left_shoulder_yaw": 30,
            "left_shoulder_pitch": 45,
            "left_elbow": -30
        })
        
        # Wait a moment to see the state changes
        print("\nWatching state changes for 3 seconds...")
        await asyncio.sleep(3)
        
        # Move to a new position
        print("\nMoving to new position...")
        await robot.move(kos, {
            "left_shoulder_yaw": -30,
            "left_shoulder_pitch": 20,
            "left_elbow": 10
        })
        
        # Wait to see more state changes
        await asyncio.sleep(3)
        
        # Access joint history if needed
        left_elbow = robot.joints["left_elbow"]
        history = left_elbow.history
        print(f"\nLeft elbow position history ({len(history)} samples):")
        for i, state in enumerate(history[-5:]):  # Show last 5 states
            print(f"  Sample {i}: position={state.position:.2f}, time={state.timestamp}")
        
        # Stop monitoring before finishing
        await robot.stop_monitoring()
        
        # Return to zero position
        print("\nReturning to home position...")
        await robot.zero_all(kos)

if __name__ == "__main__":
    asyncio.run(main())