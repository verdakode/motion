# Motion Library

A robot motion control library

## Installation

```bash
# Install from local directory
pip install -e .

# Or install from GitHub (once published)
# pip install git+https://github.com/verdakorzeniewski/motion.git
```

## Using in Other Repositories

To use this library in other repositories, you have two options:

### Option 1: Install from local directory

If you have the motion library checked out locally, you can install it in development mode in your project's virtual environment:

```bash
# Activate your project's virtual environment
source your_project_venv/bin/activate

# Install motion library in development mode
pip install -e /path/to/motion
```

### Option 2: Install from Git

Once you publish this to a Git repository, you can install it directly from Git:

```bash
pip install git+https://github.com/yourusername/motion.git
```

## Usage

```python
import asyncio
from pykos import KOS
from motion.robot import Robot

# Create robot instance
robot = Robot()

async def main():
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
        
        # Zero all joints
        await robot.zero_all(kos)

if __name__ == "__main__":
    asyncio.run(main())
```
