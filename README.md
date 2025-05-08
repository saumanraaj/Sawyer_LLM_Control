# LLM-Controlled Sawyer Robot

This repository contains a ROS-based implementation for controlling the Sawyer robot using natural language commands processed through OpenAI’s GPT models. The system allows users to issue high-level textual commands such as "move forward by 10 centimeters", which are interpreted and converted into executable robot actions via MoveIt.

The setup runs entirely in simulation using Gazebo and does not rely on visual input or object detection. This version focuses on command parsing and execution based solely on user intent and predefined spatial semantics.

---

## Repository Structure

```
ros_ws/
├── src/
│   ├── sawyer_llm_executor/         # Custom package for GPT-based control
│   │   ├── scripts/
│   │   │   ├── fake_joint_states.py        # Publishes static joint states if robot isn't broadcasting
│   │   │   ├── joint_states_relay.py       # Relays /robot/joint_states to /joint_states
│   │   │   ├── llm_command_listener.py     # Main node subscribing to user commands and triggering actions
│   │   │   ├── gpt.py                      # Handles GPT-4 API interaction and JSON parsing
│   │   │   └── sawyer_action.py            # Executes parsed commands using MoveIt
│   └── (remaining standard Sawyer SDK and Intera packages)
```

---

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Intera SDK and Sawyer simulator
- MoveIt for Sawyer
- Python 3.x
- OpenAI API Key (for GPT access)

---

## Setup Instructions

1. **Clone the Repository**
   ```bash
   git clone https://github.com/saumanraaj/Sawyer_LLM_Control.git
   cd Sawyer_LLM_Control
   ```

2. **Source the Intera Environment**
   ```bash
   source intera.sh sim
   ```

3. **Build the Workspace**
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Launch the Gazebo Simulation**
   ```bash
   roslaunch sawyer_gazebo sawyer_world.launch
   ```

5. **Publish Joint States**
   - Option A: If robot is not broadcasting joint states
     ```bash
     rosrun sawyer_llm_executor fake_joint_states.py
     ```
   - Option B: If `/robot/joint_states` exists
     ```bash
     rosrun sawyer_llm_executor joint_states_relay.py
     ```

6. **Run the GPT Command Listener**
   ```bash
   rosrun sawyer_llm_executor llm_command_listener.py
   ```

---

## Example Workflow

When the `llm_command_listener.py` node is running, publish a text command via terminal:

```bash
rostopic pub /llm/user_input std_msgs/String "data: 'move forward by 10 centimeters'"
```

The GPT model returns a structured JSON response like:

```json
{
  "actions": ["move_to"],
  "position": [0.7, 0.0]
}
```

This is then executed via the `sawyer_action.py` MoveIt interface.

---

## Notes

- The gripper is optional. If not detected, gripper-related commands will be safely ignored.
- All Cartesian movements are relative to a fixed z-height (default: 0.6m).
- The system assumes a clear workspace and does not perform collision checking beyond basic planning validation.

---

## Acknowledgements

This project builds on the official Sawyer SDK, MoveIt, and OpenAI’s GPT API. It was developed for experimental research in language-conditioned robot control.
