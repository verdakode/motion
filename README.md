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
    # Create a robot with named joints
    robot = Robot(joint_map={
        "shoulder": 1,
        "elbow": 2,
        "wrist": 3
    })
    
    # Connect to robot
    async with KOS() as kos:
        # Configure for simulator
        await robot.configure(kos, is_real=False)
        
        # Move joints to specific positions
        await robot.move(kos, {
            "shoulder": 45,  # degrees
            "elbow": 30
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

# Default configuration
robot = Robot(joint_map={"joint1": 1, "joint2": 2})

# Custom configuration
config = RobotConfig(
    sim_gains=(80, 40),      # Lower kp, kd for simulator
    real_gains=(24, 20),     # Lower gains for real robot
    max_torque=50.0,         # Limit maximum torque
    default_velocity=5.0     # Slower default movement
)
robot = Robot(joint_map={"joint1": 1, "joint2": 2}, config=config)
```

### Joint Groups

```python
# Define joint mapping
joint_map = {
    "left_shoulder": 1,
    "left_elbow": 2,
    "right_shoulder": 3,
    "right_elbow": 4
}

# Define logical groups
groups = {
    "left_arm": ["left_shoulder", "left_elbow"],
    "right_arm": ["right_shoulder", "right_elbow"],
    "shoulders": ["left_shoulder", "right_shoulder"]
}

# Create robot with groups
robot = Robot(joint_map=joint_map, groups=groups)

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