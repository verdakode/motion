#!/usr/bin/env python3
"""
Simple test script to verify the motion library can be imported correctly.
"""
import sys
import importlib.util
# Add the current directory to Python path for local testing
sys.path.insert(0, '.')

# Check if pykos is available
pykos_available = importlib.util.find_spec("pykos") is not None

try:
    # First test basic imports
    from motion.robot import Robot, RobotConfig
    from motion.robot import Joint, JointGroup, JointState
    print("✅ Successfully imported motion.robot module classes")
    
    # Mock the KOS class if pykos isn't available
    if not pykos_available:
        print("ℹ️ Note: pykos module not found, using mock KOS class for testing")
        class MockKOS:
            class actuator:
                @staticmethod
                async def configure_actuator(*args, **kwargs):
                    return None
                
                @staticmethod
                async def command_actuators(*args, **kwargs):
                    return None
                
                @staticmethod
                async def get_actuators_state(*args, **kwargs):
                    class MockResponse:
                        def __init__(self):
                            self.states = []
                    return MockResponse()
            
            async def __aenter__(self):
                return self
                
            async def __aexit__(self, *args):
                pass
        
        # Patch the Robot class to use our MockKOS
        import types
        import motion.robot.core.robot
        motion.robot.core.robot.KOS = MockKOS
    
    # Test creating objects
    robot = Robot(joint_map={"joint1": 1, "joint2": 2})
    print(f"✅ Successfully created Robot instance: {robot}")
    
    config = RobotConfig(max_torque=50.0)
    print(f"✅ Successfully created RobotConfig: {config}")
    
    joint = Joint(name="test_joint", actuator_id=1)
    print(f"✅ Successfully created Joint: {joint}")
    
    joint_group = JointGroup(name="test_group", joints=[joint])
    print(f"✅ Successfully created JointGroup: {joint_group}")
    
    state = JointState(position=1.0, velocity=0.0, torque=0.0)
    print(f"✅ Successfully created JointState: {state}")
    
    print("All tests passed! The library is ready for use in other repositories.")
except Exception as e:
    print(f"❌ Error importing or using the library: {e}")
    sys.exit(1)