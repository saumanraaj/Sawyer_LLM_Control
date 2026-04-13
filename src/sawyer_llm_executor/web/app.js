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
  div.textContent = text;
  feed.appendChild(div);
  feed.scrollTop = feed.scrollHeight;
}

function addTimeline(eventObj) {
  const div = document.createElement("div");
  div.textContent = `[${eventObj.event_id}] ${eventObj.event_type} ${JSON.stringify(eventObj.payload)}`;
  timeline.appendChild(div);
  timeline.scrollTop = timeline.scrollHeight;
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
  sessionBox.textContent = JSON.stringify(state, null, 2);

  if (state.mode !== lastMode) {
    addStoryLine(`State: ${humanizeMode(lastMode)} → ${humanizeMode(state.mode)}`);
    lastMode = state.mode;
  }

  if (state.intent_summary) {
    intentBar.textContent = `Interpreted: ${state.intent_summary}`;
    intentBar.classList.remove("hidden");
  } else {
    intentBar.classList.add("hidden");
  }

  updateActionChrome();
}

function updateActionChrome() {
  const mode = state.mode || "idle";
  quickPicks.innerHTML = "";
  quickPicks.classList.add("hidden");

  const lp = state.latest_prompt || {};
  const suggested = lp.suggested_replies || [];
  const ptype = lp.prompt_type;

  proceedBtn.classList.add("hidden");
  cancelBtn.classList.add("hidden");
  editBtn.classList.add("hidden");

  if (mode === "awaiting_warning_ack" || mode === "awaiting_confirmation") {
    proceedBtn.classList.remove("hidden");
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
    lastEventId = Math.max(lastEventId, ev.event_id);
    addTimeline(ev);
    if (ev.event_type === "prompt") {
      const cls = promptMessageClass(ev.payload);
      addMessage(ev.payload.message || JSON.stringify(ev.payload), cls);
    }
  });
}

function startEventStream() {
  try {
    const source = new EventSource(`/api/stream?since_id=${lastEventId}`);
    source.onmessage = (event) => {
      const ev = JSON.parse(event.data);
      lastEventId = Math.max(lastEventId, ev.event_id || 0);
      addTimeline(ev);
      if (ev.event_type === "prompt") {
        const cls = promptMessageClass(ev.payload);
        addMessage(ev.payload.message || JSON.stringify(ev.payload), cls);
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

async function boot() {
  addStoryLine("Console ready.");
  await fetchState();
  setInterval(fetchState, 800);
  startEventStream();
}

boot();
