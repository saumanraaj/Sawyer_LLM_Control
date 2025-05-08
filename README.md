# LLM-Controlled Sawyer Robot

This repository provides a ROS-based implementation for controlling the Sawyer robot using natural language commands interpreted by OpenAI’s GPT API. The system converts language commands into executable robot actions via MoveIt in a simulated Gazebo environment. Visual input is not used; control is based solely on textual instructions.

---

## Repository Structure

```
ros_ws/
├── src/
│   ├── sawyer_llm_executor/
│   │   ├── scripts/
│   │   │   ├── fake_joint_states.py
│   │   │   ├── joint_states_relay.py
│   │   │   ├── llm_command_listener.py
│   │   │   ├── gpt.py
│   │   │   └── sawyer_action.py
│   └── (standard Intera SDK, simulator, and MoveIt packages)
```

---

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- Intera SDK and Sawyer Simulator
- MoveIt for Sawyer
- Python 3.x
- OpenAI API Key (exported via environment variable `OPENAI_API_KEY`)

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

---

## Launch Sequence (Strict Order)

1. **Launch the Gazebo World with Sawyer**
   ```bash
   roslaunch sawyer_gazebo sawyer_world.launch
   ```

2. **Start the Intera Interface**
   ```bash
   rosrun intera_interface joint_trajectory_action_server.py
   ```

3. **Launch Sawyer MoveIt Configuration**
   ```bash
   roslaunch sawyer_moveit_config sawyer_moveit.launch
   ```

4. **Publish Joint States**
   - If robot is not publishing `/robot/joint_states`, use:
     ```bash
     rosrun sawyer_llm_executor fake_joint_states.py
     ```
   - Otherwise, relay the joint states:
     ```bash
     rosrun sawyer_llm_executor joint_states_relay.py
     ```

5. **Run the GPT Command Listener**
   ```bash
   rosrun sawyer_llm_executor llm_command_listener.py
   ```

---

## Sending Commands

Once the GPT listener node is active, you can publish text commands:

```bash
rostopic pub /llm/user_input std_msgs/String "data: 'move forward by 10 centimeters'"
```

The system will:
1. Send the command to the GPT API
2. Parse the returned JSON structure
3. Use MoveIt to execute corresponding actions

---

## Notes

- Default z-height is 0.6 meters.
- Gripper actions (open/close) will be ignored if the robot does not detect a gripper.
- The MoveIt planner handles IK and motion; collision avoidance is not guaranteed.
- Requires a valid OpenAI GPT API key via `OPENAI_API_KEY`.

---

## Acknowledgements

This project builds upon the official Rethink Robotics Sawyer SDK, Gazebo simulation environment, MoveIt, and OpenAI's GPT models.
