# Sawyer LLM Executor - Setup and Usage Guide

This guide provides step-by-step instructions to set up and use the LLM-controlled Sawyer robot system.

## Prerequisites

1. **OpenAI API Key**: Set your API key (already configured in `.bashrc`)
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

2. **Network Configuration**: Ensure `intera.sh` is configured with:
   - Robot IP: `192.168.0.103`
   - Your computer IP: `192.168.0.105`

3. **Robot in SDK mode**: The Sawyer must be booted in **Intera-SDK** mode (not Intera-only). Only then does the ROS master run on port 11311. If you only see Intera Studio at `http://<robot-ip>:3000` and port 11311 is not open, switch the robot to SDK mode:
   - Connect a **USB keyboard** to the robot controller.
   - **Power off** the robot completely (fan and screen off), wait 5 seconds, then power on.
   - As it boots, when you see **"Preparing to boot Intera…"** on the screen, press **Ctrl+F** repeatedly until the **Field Service Menu (FSM)** appears.
   - In the FSM, use the **arrow keys** to select **"Intera-SDK"** (not "Next boot in: Intera").
   - Accept the disclaimer (tab to "Yes", Enter), then choose **Reboot**.
   - After reboot the robot runs in SDK mode: ROS master is on port 11311 and you can use `enable_robot.py`, `intera.sh`, etc. Intera Studio (port 3000) may still be available in some setups.

## Quick Robot Test (Optional)

Before starting LLM control, verify the robot works:

```bash
cd /home/sauman25/ros_ws
source intera.sh
rosrun intera_interface enable_robot.py -e
rosrun intera_examples go_to_joint_angles.py -q 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0
```

If you see "Motion controller successfully finished the trajectory!", the robot is working.

## LLM Control Setup Steps

Start 5 terminals in order and wait for each to fully initialize before starting the next.

### Terminal 1 - Enable Robot

```bash
cd /home/sauman25/ros_ws
source intera.sh
rosrun intera_interface enable_robot.py -e
```

**Wait for**: `[INFO] Robot Enabled`

### Terminal 2 - Start Joint Trajectory Action Server

```bash
cd /home/sauman25/ros_ws
source intera.sh
rosrun intera_interface joint_trajectory_action_server.py
```

**Wait for**: `Joint Trajectory Action Server Running. Ctrl-c to quit`

### Terminal 3 - Start MoveIt

```bash
cd /home/sauman25/ros_ws
source intera.sh
roslaunch sawyer_moveit_config sawyer_moveit.launch
```

**Wait for**: `Ready to take commands for planning group right_arm`

### Terminal 4 - Joint States Relay (required for executor)

With the **real robot**, the robot publishes `/robot/joint_states`; the executor expects `/joint_states`:

```bash
cd /home/sauman25/ros_ws
source intera.sh
rosrun sawyer_llm_executor joint_states_relay.py
```

**Leave this running.** You should see: `Relaying /robot/joint_states to /joint_states...`

- **No robot (sim/fake only)**: Run `rosrun sawyer_llm_executor fake_joint_states.py` first, then the relay.

### Terminal 5 - Start LLM Command Listener

```bash
cd /home/sauman25/ros_ws
source intera.sh
export OPENAI_API_KEY="your-api-key"
rosrun sawyer_llm_executor llm_command_listener.py
```

**Wait for**: `Listening for LLM user commands...`

### Terminal 6 - Send Commands

```bash
cd /home/sauman25/ros_ws
source intera.sh
rostopic pub -1 /llm/user_input std_msgs/String "data: 'move forward by 10 centimeters'"
```

## Example Commands

```bash
# Move forward
rostopic pub -1 /llm/user_input std_msgs/String "data: 'move forward by 10 centimeters'"

# Move right
rostopic pub -1 /llm/user_input std_msgs/String "data: 'move right by 20 centimeters'"

# Move to specific position
rostopic pub -1 /llm/user_input std_msgs/String "data: 'move to position (0.5, 0.1)'"

# Move backward
rostopic pub -1 /llm/user_input std_msgs/String "data: 'move backward by 5 centimeters'"

# Open gripper
rostopic pub -1 /llm/user_input std_msgs/String "data: 'open gripper'"

# Close gripper
rostopic pub -1 /llm/user_input std_msgs/String "data: 'close gripper'"
```

## System Status Checklist

After setup, verify:

- ✓ Robot enabled (`enable_robot.py -e` shows "Robot Enabled")
- ✓ Joint trajectory action server running (Terminal 2 shows "Running")
- ✓ MoveIt running (Terminal 3 shows "Ready to take commands")
- ✓ Joint states available (`rostopic echo /joint_states` shows data)
- ✓ LLM listener running (Terminal 5 shows "Listening for LLM user commands...")

## Troubleshooting

### CONTROL_FAILED Error

If you see `ABORTED: CONTROL_FAILED`:
1. Ensure robot is enabled: `rosrun intera_interface enable_robot.py -e`
2. Ensure action server is running: `rosrun intera_interface joint_trajectory_action_server.py`
3. Restart MoveIt after starting the action server

### No Joint States

If joint states aren't available:
```bash
rosrun sawyer_llm_executor joint_states_relay.py
```

### Robot Not Moving

1. Check robot is enabled: `rosrun intera_interface enable_robot.py -s`
2. Verify action server: `rostopic list | grep follow_joint_trajectory`
3. Check MoveIt is ready: Look for "Ready to take commands" message

## Shutdown

To stop the system:
1. Press `Ctrl+C` in each terminal (in reverse order: 5, 4, 3, 2, 1)
2. Optionally disable robot: `rosrun intera_interface enable_robot.py -d`

## Notes

- The robot must be enabled before commands will execute
- The joint trajectory action server must be running before MoveIt can execute plans
- MoveIt should be started after the action server to ensure proper connection
- No gripper is required - the system works without it (gripper commands will be skipped)
