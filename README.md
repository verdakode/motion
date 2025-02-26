# Motion Library

A high-level Python library for robot motion control with a focus on simplicity and convention over configuration.

## Features

- **Simple Joint Control**: Map actuator IDs to human-readable joint names
- **Joint Grouping**: Organize joints into logical groups for easier management
- **Configuration Management**: Handle different configurations for simulated and real robots
- **Asynchronous API**: Built with Python's asyncio for efficient operation
- **State Tracking**: Access joint positions, velocities, and torques

## Requirements

- Python ≥ 3.8
- [PyKOS](https://github.com/example/pykos) for robot communication

## Installation

```bash
# Install from local directory for development
pip install -e .

# Or install from GitHub (once published)
pip install git+https://github.com/verdakorzeniewski/motion.git
```

## Quick Start

```python
import asyncio
from pykos import KOS
from motion.robot import Robot

async def main():
    # Create a robot using the default joint mapping
    robot = Robot()
    
    # Connect to robot
    async with KOS() as kos:
        # Configure for simulator
        await robot.configure(kos, is_real=False)
        
        # Move joints to specific positions
        await robot.move(kos, {
            "left_shoulder_pitch": 45,  # degrees
            "left_elbow": 30
        })
        
        # Get current joint states
        states = await robot.get_states(kos)
        for name, state in states.items():
            print(f"{name}: pos={state.position:.2f}, vel={state.velocity:.2f}")
        
        # Return to home position
        await robot.zero_all(kos)

if __name__ == "__main__":
    asyncio.run(main())
```

## Key Concepts

### Robot Configuration

```python
from motion.robot import Robot, RobotConfig

# Use with default joint mapping and configuration
robot = Robot()

# Custom configuration
config = RobotConfig(
    sim_gains=(80, 40),      # Lower kp, kd for simulator
    real_gains=(24, 20),     # Lower gains for real robot
    max_torque=50.0,         # Limit maximum torque
    default_velocity=5.0     # Slower default movement
)
robot = Robot(config=config)
```

### Joint Groups

```python
# Define logical groups using the default joint names
groups = {
    "left_arm": ["left_shoulder_yaw", "left_shoulder_pitch", "left_elbow"],
    "right_arm": ["right_shoulder_yaw", "right_shoulder_pitch", "right_elbow"],
    "grippers": ["left_gripper", "right_gripper"]
}

# Create robot with default joint mapping and custom groups
robot = Robot(groups=groups)

# You can also specify a custom joint mapping if needed
custom_joint_map = {
    "base": 1,
    "arm": 2,
    "wrist": 3
}
custom_robot = Robot(joint_map=custom_joint_map)

# Work with a specific group
left_arm = robot.get_group("left_arm")
print(f"Left arm has {len(left_arm)} joints")
```

## Core Classes

- **Robot**: Main interface for controlling joints and groups
- **Joint**: Represents a single robot joint with name and actuator ID
- **JointState**: Holds position, velocity, and torque data
- **JointGroup**: Collection of joints that can be controlled together
- **RobotConfig**: Configuration settings for joint control parameters

## Examples

Check the `examples/` directory for more usage examples:

- `minimal.py`: Basic usage with minimal setup
- `basic_usage.py`: More complete example with joint groups

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.