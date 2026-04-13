#!/usr/bin/env bash
set -euo pipefail

# Scripted publish commands for HRI interaction demos.
# Usage:
#   source intera.sh
#   rosparam set /llm_executor_node/enable_interaction_layer true
#   rosrun sawyer_llm_executor llm_command_listener.py
#   ./test_interaction_scenarios.sh

publish_cmd () {
  local text="$1"
  echo "Publishing: $text"
  rostopic pub -1 /llm/user_input std_msgs/String "data: '$text'"
  sleep 1
}

publish_reply () {
  local text="$1"
  echo "Replying: $text"
  rostopic pub -1 /llm/user_response std_msgs/String "data: '$text'"
  sleep 1
}

echo "Scenario 1: Baseline underspecified command"
publish_cmd "move forward a little"

echo "Scenario 2: Clarification + edit"
publish_cmd "move left"
publish_reply "edit: move left by 10 cm"
publish_reply "confirm"

echo "Scenario 3: Risk warning + safe recovery"
publish_cmd "move down by 20 cm"
publish_reply "edit: move down by 3 cm"
publish_reply "confirm"

echo "Scenario 4: Safe direct execution"
publish_cmd "move forward by 5 cm"
