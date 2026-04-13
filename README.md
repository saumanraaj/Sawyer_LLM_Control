# Sawyer LLM Control

ROS **Noetic** workspace for controlling a **Rethink Sawyer** with **natural language**. Commands are published on `/llm/user_input`; the `sawyer_llm_executor` node calls the OpenAI API (optional for simple relative moves), parses actions, and drives the arm through **MoveIt** (`right_arm`).

Also includes **OptiTrack** helpers under `optitrack/` and a bundled `ros1_ws/` tree used in this lab setup.

## What you need

- Ubuntu 20.04 and ROS Noetic  
- Intera SDK and Sawyer in **Intera-SDK** mode (ROS master on the robot, typically port `11311`)  
- MoveIt packages for Sawyer in this workspace  
- `OPENAI_API_KEY` in the environment if you use GPT-backed parsing for complex commands    

Configure robot and PC IPs in `intera.sh` (and set `ROS_IP` to your PC’s address on the robot subnet if MoveIt nodes fail to find each other).

## Run on the real robot (typical order)

Use a **separate terminal** for each step. In every terminal:

```bash
cd <path-to-this-repo> # catkin workspace root
source intera.sh
export ROS_IP=<your-pc-ip-on-robot-lan>   # e.g. 192.168.0.105
unset ROS_HOSTNAME
```

Then start, in order:

1. **Enable arm**
   ```bash
   rosrun intera_interface enable_robot.py -e
   ```

2. **Joint trajectory action server**
   ```bash
   rosrun intera_interface joint_trajectory_action_server.py
   ```

3. **MoveIt**
   ```bash
   roslaunch sawyer_moveit_config sawyer_moveit.launch
   ```
   Wait until you see something like: `Ready to take commands for planning group right_arm`.

4. **Joint states relay** (robot publishes `/robot/joint_states`; MoveIt expects `/joint_states`)
   ```bash
   rosrun sawyer_llm_executor joint_states_relay.py
   ```

5. **LLM listener**
   ```bash
   export OPENAI_API_KEY="your-key-here"
   rosrun sawyer_llm_executor llm_command_listener.py
   ```
   Wait for: `Listening for LLM user commands...`

6. **Send a command** (another terminal, same `source intera.sh` + `ROS_IP` if needed)
   ```bash
   rostopic pub -1 /llm/user_input std_msgs/String "data: 'move forward by 10 centimeters'"
   ```

## Behavior notes

- Simple phrases like *move forward / back / left / right / up / down* with cm or m are parsed locally as **relative** moves in the planning frame; other wording may still go through GPT.  
- Cartesian targets are checked against a **workspace box** in code; targets outside it are rejected with a clear log instead of planning forever.  

## Build

```bash
cd <path-to-this-repo>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## More detail

See `LLM_CONTROL_README.md` for SDK boot, troubleshooting, and simulator-oriented steps.
