const feed = document.getElementById("feed");
const timeline = document.getElementById("timeline");
const story = document.getElementById("story");
const textInput = document.getElementById("textInput");
const sendBtn = document.getElementById("sendBtn");
const proceedBtn = document.getElementById("proceedBtn");
const cancelBtn = document.getElementById("cancelBtn");
const editBtn = document.getElementById("editBtn");
const statusChip = document.getElementById("statusChip");
const sessionBox = document.getElementById("sessionBox");
const intentBar = document.getElementById("intentBar");
const quickPicks = document.getElementById("quickPicks");
const editModal = document.getElementById("editModal");
const editArea = document.getElementById("editArea");
const editReason = document.getElementById("editReason");
const editSubmit = document.getElementById("editSubmit");
const editClose = document.getElementById("editClose");

let state = { mode: "idle" };
let lastEventId = 0;
let lastMode = "idle";
const renderedEventIds = new Set();
const INTERACTIVE_MODES = new Set([
  "awaiting_clarification",
  "awaiting_confirmation",
  "awaiting_warning_ack",
  "editing",
]);

function humanizeMode(mode) {
  const m = {
    idle: "Idle",
    parsing: "Thinking…",
    awaiting_clarification: "Needs detail",
    awaiting_confirmation: "Preview",
    awaiting_warning_ack: "Safety check",
    ready_to_execute: "Ready",
    executing: "Moving",
    completed: "Done",
    editing: "You revised the command",
    cancelled: "Stopped",
    timeout: "Timed out",
  };
  return m[mode] || mode || "…";
}

function addStoryLine(text) {
  const div = document.createElement("div");
  div.textContent = `${new Date().toLocaleTimeString()} — ${text}`;
  story.appendChild(div);
  story.scrollTop = story.scrollHeight;
}

function addMessage(text, cssClass) {
  const div = document.createElement("div");
  div.className = `msg ${cssClass}`;
  div.dataset.kind = cssClass === "user" ? "user" : "prompt";
  div.textContent = text;
  feed.appendChild(div);
  feed.scrollTop = feed.scrollHeight;
}

function addTimeline(eventObj) {
  const wrap = document.createElement("div");
  wrap.className = "timeline-item";

  const header = document.createElement("div");
  header.className = "timeline-header";

  const badge = document.createElement("span");
  badge.className = `timeline-badge ${eventObj.event_type || "unknown"}`;
  badge.textContent = `#${eventObj.event_id} ${eventObj.event_type || "event"}`;

  const summary = document.createElement("span");
  summary.className = "timeline-summary";
  summary.textContent = summarizeEvent(eventObj);

  header.appendChild(badge);
  header.appendChild(summary);
  wrap.appendChild(header);

  const details = document.createElement("details");
  details.className = "timeline-details";

  const detailsSummary = document.createElement("summary");
  detailsSummary.textContent = "Details";

  const payload = document.createElement("pre");
  payload.textContent = JSON.stringify(eventObj.payload || {}, null, 2);

  details.appendChild(detailsSummary);
  details.appendChild(payload);
  wrap.appendChild(details);

  timeline.appendChild(wrap);
  timeline.scrollTop = timeline.scrollHeight;
}

function summarizeEvent(eventObj) {
  const payload = eventObj.payload || {};
  const et = eventObj.event_type || "";
  if (et === "ui_command") return payload.text || "User command sent";
  if (et === "prompt") return payload.message || `${payload.prompt_type || "Prompt"} requested`;
  if (et === "state") {
    const mode = payload.mode ? humanizeMode(payload.mode) : "State update";
    const cmd = payload.command ? `: ${payload.command}` : "";
    return `${mode}${cmd}`;
  }
  if (et === "response" || et === "ui_response") return payload.response || "User response sent";
  return JSON.stringify(payload);
}

function promptMessageClass(payload) {
  const t = (payload && payload.prompt_type) || "system";
  const cat = (payload && payload.issue_category) || "";
  if (t === "warning" || cat === "risk") return "risk";
  if (t === "clarification" || cat === "ambiguity") return "ambiguity";
  if (cat === "mixed") return "mixed";
  if (t === "preview") return "preview";
  return "system";
}

function setState(newState) {
  state = newState;
  statusChip.textContent = humanizeMode(state.mode);
  renderSession(state);

  if (state.mode !== lastMode) {
    addStoryLine(`State: ${humanizeMode(lastMode)} → ${humanizeMode(state.mode)}`);
    lastMode = state.mode;
  }

  if (state.intent_summary) {
    intentBar.textContent = `In plain words: ${state.intent_summary}`;
    intentBar.classList.remove("hidden");
  } else {
    intentBar.classList.add("hidden");
  }

  updateActionChrome();
}

function renderSession(s) {
  const showPrompt = INTERACTIVE_MODES.has(s.mode);
  const lp = showPrompt ? (s.latest_prompt || null) : null;
  const suggestions = (lp && lp.suggested_replies) || [];
  const modeLabel = humanizeMode(s.mode);

  const parts = [];
  parts.push(`<div class="session-row"><span class="k">Mode</span><span class="v">${escapeHtml(modeLabel)}</span></div>`);
  parts.push(`<div class="session-row"><span class="k">Request</span><span class="v">${escapeHtml(s.request_id || "—")}</span></div>`);
  parts.push(`<div class="session-row"><span class="k">Turn</span><span class="v">${escapeHtml(String(s.turn ?? 0))}</span></div>`);
  parts.push(`<div class="session-row"><span class="k">Command</span><span class="v">${escapeHtml(s.command || "—")}</span></div>`);

  if (lp) {
    parts.push(`<div class="session-section">Prompt</div>`);
    parts.push(`<div class="session-row"><span class="k">Type</span><span class="v">${escapeHtml(lp.prompt_type || "—")}</span></div>`);
    parts.push(`<div class="session-row"><span class="k">Issue</span><span class="v">${escapeHtml(lp.issue_category || "—")}</span></div>`);
    parts.push(`<div class="session-row"><span class="k">Short summary</span><span class="v">${escapeHtml(lp.intent_summary || "—")}</span></div>`);
    parts.push(`<div class="session-note">${escapeHtml(lp.message || "")}</div>`);

    if (suggestions.length) {
      const chips = suggestions
        .map(
          (x) =>
            `<button type="button" class="session-chip session-suggestion" data-suggestion="${escapeHtml(x)}">${escapeHtml(x)}</button>`
        )
        .join("");
      parts.push(`<div class="session-section">Suggestions</div>`);
      parts.push(`<div class="session-chip-wrap">${chips}</div>`);
    }
  } else {
    parts.push(`<div class="session-section">Prompt</div>`);
    parts.push(`<div class="session-row"><span class="k">Status</span><span class="v">No pending prompt</span></div>`);
  }

  sessionBox.innerHTML = parts.join("");
}

function escapeHtml(text) {
  return String(text)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");
}

function lastFeedUserLine() {
  const msgs = feed.querySelectorAll('.msg[data-kind="user"]');
  if (!msgs.length) return null;
  const t = msgs[msgs.length - 1].textContent || "";
  if (t.startsWith("You: ")) return t.slice(5);
  return t;
}

function appendUserFeedLine(text) {
  const line = `You: ${text}`;
  if (lastFeedUserLine() === text) return;
  addMessage(line, "user");
}

function appendPromptFeedLine(payload) {
  const cls = promptMessageClass(payload);
  addMessage(payload.message || JSON.stringify(payload), cls);
}

function updateActionChrome() {
  const mode = state.mode || "idle";
  quickPicks.innerHTML = "";
  quickPicks.classList.add("hidden");

  const lp = state.latest_prompt || {};
  const suggested = lp.suggested_replies || [];
  const ptype = lp.prompt_type;
  const allowed = lp.allowed_actions;
  const canProceed =
    allowed == null || allowed.length === 0
      ? mode !== "awaiting_warning_ack"
      : allowed.includes("proceed");

  proceedBtn.classList.add("hidden");
  cancelBtn.classList.add("hidden");
  editBtn.classList.add("hidden");

  if (mode === "awaiting_warning_ack" || mode === "awaiting_confirmation") {
    if (canProceed) proceedBtn.classList.remove("hidden");
    cancelBtn.classList.remove("hidden");
    editBtn.classList.remove("hidden");
  } else if (mode === "awaiting_clarification") {
    editBtn.classList.remove("hidden");
    cancelBtn.classList.remove("hidden");
    if (suggested.length && ptype === "clarification") {
      quickPicks.classList.remove("hidden");
      suggested.forEach((label) => {
        const b = document.createElement("button");
        b.type = "button";
        b.textContent = label;
        b.addEventListener("click", () => sendResponse(label));
        quickPicks.appendChild(b);
      });
    }
  }
}

async function fetchState() {
  const res = await fetch("/api/state");
  const json = await res.json();
  setState(json);
}

async function fetchEvents() {
  const res = await fetch(`/api/events?since_id=${lastEventId}`);
  const json = await res.json();
  (json.events || []).forEach((ev) => {
    if (renderedEventIds.has(ev.event_id)) return;
    renderedEventIds.add(ev.event_id);
    lastEventId = Math.max(lastEventId, ev.event_id);
    addTimeline(ev);
    if (ev.event_type === "ui_command" && ev.payload && ev.payload.text) {
      appendUserFeedLine(ev.payload.text);
    }
    if (ev.event_type === "ui_response" && ev.payload && ev.payload.response) {
      appendUserFeedLine(ev.payload.response);
    }
    if (ev.event_type === "prompt") {
      appendPromptFeedLine(ev.payload || {});
    }
  });
}

function startEventStream() {
  try {
    const source = new EventSource(`/api/stream?since_id=${lastEventId}`);
    source.onmessage = (event) => {
      const ev = JSON.parse(event.data);
      if (renderedEventIds.has(ev.event_id)) return;
      renderedEventIds.add(ev.event_id);
      lastEventId = Math.max(lastEventId, ev.event_id || 0);
      addTimeline(ev);
      if (ev.event_type === "ui_command" && ev.payload && ev.payload.text) {
        appendUserFeedLine(ev.payload.text);
      }
      if (ev.event_type === "ui_response" && ev.payload && ev.payload.response) {
        appendUserFeedLine(ev.payload.response);
      }
      if (ev.event_type === "prompt") {
        appendPromptFeedLine(ev.payload || {});
      }
    };
    source.onerror = () => {
      source.close();
      setInterval(fetchEvents, 500);
    };
  } catch (_e) {
    setInterval(fetchEvents, 500);
  }
}

async function sendCommand(text) {
  await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ text }),
  });
  addMessage(`You: ${text}`, "user");
  addStoryLine(`Command: ${text}`);
}

async function sendResponse(text) {
  await fetch("/api/respond", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ response: text }),
  });
  addMessage(`You: ${text}`, "user");
  addStoryLine(`Response: ${text}`);
}

async function sendPayload(text) {
  const mode = state.mode || "idle";
  const idleLike = ["idle", "completed", "cancelled", "timeout"].includes(mode);
  if (idleLike) {
    await sendCommand(text);
    return;
  }
  await sendResponse(text);
}

sendBtn.addEventListener("click", async () => {
  const text = textInput.value.trim();
  if (!text) return;
  textInput.value = "";
  await sendPayload(text);
});

textInput.addEventListener("keydown", async (e) => {
  if (e.key === "Enter") {
    sendBtn.click();
  }
});

proceedBtn.addEventListener("click", async () => sendResponse("proceed"));
cancelBtn.addEventListener("click", async () => sendResponse("cancel"));
editBtn.addEventListener("click", () => {
  const lp = state.latest_prompt || {};
  editReason.textContent = lp.message
    ? "Use a full phrase with distance (e.g. move left by 10 cm)."
    : "Enter a clearer command.";
  editArea.value = (lp.original_command || state.command || "").trim();
  editModal.classList.remove("hidden");
});

editClose.addEventListener("click", () => editModal.classList.add("hidden"));
editSubmit.addEventListener("click", async () => {
  const t = editArea.value.trim();
  if (!t) return;
  editModal.classList.add("hidden");
  await sendResponse(t);
});

sessionBox.addEventListener("click", async (e) => {
  const target = e.target.closest(".session-suggestion");
  if (!target) return;
  const text = (target.dataset.suggestion || "").trim();
  if (!text) return;
  await sendResponse(text);
});

async function boot() {
  addStoryLine("Console ready.");
  await fetchState();
  await fetchEvents();
  setInterval(fetchState, 800);
  startEventStream();
}

boot();
