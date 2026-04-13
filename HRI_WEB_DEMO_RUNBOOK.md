# Sawyer Web HRI Demo Runbook

## 1) Launch order

1. Start robot backend stack (existing flow):
   - enable robot
   - joint trajectory action server
   - MoveIt
   - joint states relay
2. Enable interaction layer:
   - `rosparam set /enable_interaction_layer true`
3. Start listener:
   - `rosrun sawyer_llm_executor llm_command_listener.py`
4. Start web server:
   - `python3 /home/sauman25/ros_ws/src/sawyer_llm_executor/scripts/hri_web_server.py`
5. Open UI:
   - `http://localhost:8010/`

## 2) Acceptance checks

- Interaction state appears in UI header (not stuck at unknown).
- Command input works from UI (no terminal `rostopic pub` needed).
- Prompt cards appear for clarification/warning/preview.
- **Proceed** / **Stop** / **Revise** send responses on `/llm/user_response` (or type `proceed` / `cancel` / a full revised phrase).
- Clarification accepts **plain text** repairs (e.g. `move left by 3 cm`), not only `edit:`.
- Clear commands with no ambiguity/risk issues **execute without** an extra confirmation step.
- Legacy mode remains available by disabling:
  - `rosparam set /enable_interaction_layer false`

## 3) Demo scenarios

### Scenario A: Underspecified command clarification
- Send: `move left`
- Expect:
  - state: `awaiting_clarification`
  - clarification prompt card
  - no arm motion before user response
- Respond: `edit: move left by 10 centimeters`
- Confirm and execute.

### Scenario B: Safety warning
- Send: `move down by 20 centimeters`
- Expect warning card + explicit user action required.
- Respond with safer edit:
  - `edit: move down by 3 centimeters`
- Confirm and execute.

### Scenario C: Fast-path safe command
- Send: `move forward by 5 centimeters`
- Expect direct preview/confirmation or execute path per policy.

## 4) Logs to collect

- Backend interaction log:
  - `/home/sauman25/ros_ws/logs/interaction_events.jsonl`
- Web UI log:
  - `/home/sauman25/ros_ws/logs/hri_ui_events.jsonl`

## 5) Troubleshooting

- If UI does not update:
  - confirm `llm_command_listener.py` prints `Interaction layer enabled: True`
  - check `/llm/system_prompt_json` and `/llm/interaction_state` topics exist
- If command executes without clarification:
  - verify command contains no explicit distance and ambiguity rules are active
- If responses are ignored:
  - ensure replies come from UI (`/api/respond`) or `/llm/user_response`
