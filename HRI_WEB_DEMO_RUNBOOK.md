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

## 2) Interaction layer: cases and policy

The listener uses **UncertaintyEngine** plus **InteractionPolicy**. Resolution order is: **WARN → CLARIFY → PREVIEW_CONFIRM → EXECUTE** (first match wins).

Configurable defaults (see `uncertainty_engine.py`): e.g. `displacement_preview_m` **0.15** m, `displacement_warn_m` **0.25** m, `downward_warn_m` **0.08** m, `low_conf_threshold` **0.65**.

### WARN (safety — revise or cancel; **Proceed** is not offered for unsafe plans)

| Issue type | Meaning |
|------------|---------|
| **WORKSPACE_LIMIT_RISK** | Planned end pose after `move_relative` / `move_to` is outside the Cartesian box in `sawyer_action.workspace_limits`. Suggestion chips favor **opposite / orthogonal** moves, not repeating the failing direction. |
| **DOWNWARD_RISK** | Relative move has downward **dz** magnitude **>** `downward_warn_m` (default **0.08** m). |
| **OVERSIZED_DISPLACEMENT** | Total displacement magnitude **>** `displacement_warn_m` (default **0.25** m). |

### CLARIFY (ambiguous or incomplete — user must supply a clearer command)

| Issue type | Meaning |
|------------|---------|
| **MISSING_MAGNITUDE** | Direction words (**left**, **forward**, etc.) without a **numeric distance + unit** in the text, or structured `move_relative` missing offsets. |
| **VAGUE_MAGNITUDE** | Parser marked a **vague** size (e.g. “a little”). |
| **AMBIGUOUS_FRAME** | Operation has no explicit **frame** in structured intent. |
| **LOW_PARSE_CONFIDENCE** | `parser_meta.parse_confidence` **<** threshold (default **0.65**). |
| **UNSPECIFIED_Z_FOR_MOVE_TO** | `move_to` target missing **z** in structured pose. |

### PREVIEW_CONFIRM (confirm before executing)

| Issue type | Meaning |
|------------|---------|
| **LARGE_BUT_ALLOWED_DISPLACEMENT** | Displacement **≥** `displacement_preview_m` (default **0.15** m) but not over the oversized warning — e.g. a **15 cm** step often triggers preview. |
| **MULTI_STEP_PREVIEW** | More than one operation in the plan (multi-step sequence). |

### EXECUTE

No WARN/CLARIFY/PREVIEW issue applies; motion runs per policy (interaction layer still active for state/publish).

### UI issue category (Session panel)

- **risk** — WARN family  
- **ambiguity** — CLARIFY family  
- **mixed** — both present  
- **none** — preview-only or clear execute path  

Lower layers (MoveIt, joint limits, IK failures, gripper hardware) are **not** modeled as these issue types; they surface at execution time.

## 3) Acceptance checks

- Interaction state appears in UI header (not stuck at unknown).
- Command input works from UI (no terminal `rostopic pub` needed).
- Prompt cards appear for clarification/warning/preview.
- **Proceed** / **Stop** / **Revise** send responses on `/llm/user_response` where allowed (**WARN** safety prompts do **not** offer Proceed; revise or cancel instead).
- Clarification accepts **plain text** repairs (e.g. `move left by 3 cm`), not only `edit:`.
- Clear commands with no ambiguity/risk issues **execute without** an extra confirmation step.
- Legacy mode remains available by disabling:
  - `rosparam set /enable_interaction_layer false`

## 4) Demo scenarios

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

## 5) Logs to collect

- Backend interaction log:
  - `/home/sauman25/ros_ws/logs/interaction_events.jsonl`
- Web UI log:
  - `/home/sauman25/ros_ws/logs/hri_ui_events.jsonl`

## 6) Troubleshooting

- If UI does not update:
  - confirm `llm_command_listener.py` prints `Interaction layer enabled: True`
  - check `/llm/system_prompt_json` and `/llm/interaction_state` topics exist
- If command executes without clarification:
  - verify command contains no explicit distance and ambiguity rules are active
- If responses are ignored:
  - ensure replies come from UI (`/api/respond`) or `/llm/user_response`
