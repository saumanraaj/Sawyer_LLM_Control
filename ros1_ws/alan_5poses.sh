#!/usr/bin/env bash
set -euo pipefail

# ----------------------------
# 0) Source ROS + Intera env
# ----------------------------
# Adjust these paths if your workspace is different.
source /opt/ros/noetic/setup.bash

# Your ROS1 workspace (the one that contains intera_sdk / intera_examples)
WS="$HOME/ros_ws/ros1_ws"

if [ -f "$WS/devel/setup.bash" ]; then
  source "$WS/devel/setup.bash"
else
  echo "[ERROR] Cannot find: $WS/devel/setup.bash"
  echo "Did you catkin_make in $WS ?"
  exit 1
fi

# Intera environment (usually sets ROS_MASTER_URI, robot hostname, etc.)
# If you normally do: source intera.sh
if [ -f "$WS/intera.sh" ]; then
  source "$WS/intera.sh"
elif [ -f "$WS/src/intera_sdk/intera.sh" ]; then
  source "$WS/src/intera_sdk/intera.sh"
else
  echo "[ERROR] Cannot find intera.sh in expected locations."
  exit 1
fi

# ----------------------------
# 1) Settings
# ----------------------------
DELAY_SEC=3
SPEED=0.2     # 0.001 .. 1.0
ACCEL=0.2     # 0.001 .. 1.0
TIMEOUT=20    # seconds (optional)

# ----------------------------
# 2) 5 poses (EDIT THESE)
# Each pose = 7 joint angles (J0..J6)
# ----------------------------
POSE1=( 0.00 -0.90  0.00  1.80  0.00 -0.90  0.00 )
POSE2=( 0.20 -0.80  0.10  1.60 -0.10 -0.70  0.20 )
POSE3=( 0.40 -0.60  0.20  1.40 -0.20 -0.50  0.40 )
POSE4=( 0.20 -0.80  0.10  1.60 -0.10 -0.70  0.20 )
POSE5=( 0.00 -0.90  0.00  1.80  0.00 -0.90  0.00 )

POSES=(POSE1 POSE2 POSE3 POSE4 POSE5)

# ----------------------------
# 3) Helper to run one pose
# ----------------------------
move_pose () {
  local pose_name="$1"
  shift
  local joints=("$@")

  echo ""
  echo "=== Moving to ${pose_name}: ${joints[*]} ==="

  # go_to_joint_angles.py lives in intera_examples
  rosrun intera_examples go_to_joint_angles.py \
    -q "${joints[@]}" \
    -s "${SPEED}" \
    -a "${ACCEL}" \
    --timeout "${TIMEOUT}"
}

# ----------------------------
# 4) Run sequence
# ----------------------------
echo "Starting 5-pose sequence with ${DELAY_SEC}s delay between poses..."
for p in "${POSES[@]}"; do
  # Indirect expansion: get array contents by name
  joints=("${!p[@]}")
  move_pose "$p" "${joints[@]}"
  echo "Sleeping ${DELAY_SEC}s..."
  sleep "${DELAY_SEC}"
done

echo "Done."

