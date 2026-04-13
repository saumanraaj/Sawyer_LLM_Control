#!/usr/bin/env python3

import asyncio
import json
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from hri_ros_bridge import HRIRosBridge
from hri_session_store import HRISessionStore
from hri_ui_logger import HRIUILogger


ROOT_DIR = Path(__file__).resolve().parent.parent
WEB_DIR = ROOT_DIR / "web"

app = FastAPI(title="Sawyer HRI Web Console")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

store = HRISessionStore()
bridge = HRIRosBridge(store=store)
ui_logger = HRIUILogger("/home/sauman25/ros_ws/logs/hri_ui_events.jsonl")


class CommandRequest(BaseModel):
    text: str


class ResponseRequest(BaseModel):
    response: str


@app.on_event("startup")
def on_startup():
    bridge.start()
    ui_logger.log("server_start", {"message": "HRI web server started"})


@app.get("/health")
def health():
    return {"ok": True}


@app.post("/api/command")
def api_command(req: CommandRequest):
    text = req.text.strip()
    if not text:
        raise HTTPException(status_code=400, detail="Command text is empty.")
    bridge.submit_command(text)
    ui_logger.log("api_command", {"text": text})
    return {"accepted": True}


@app.post("/api/respond")
def api_respond(req: ResponseRequest):
    response = req.response.strip()
    if not response:
        raise HTTPException(status_code=400, detail="Response text is empty.")
    bridge.submit_response(response)
    ui_logger.log("api_respond", {"response": response})
    return {"accepted": True}


@app.get("/api/state")
def api_state():
    return bridge.get_state()


@app.get("/api/events")
def api_events(since_id: int = 0):
    events = bridge.get_events(since_id=since_id)
    return {"events": events}


@app.get("/api/stream")
async def api_stream(since_id: int = 0):
    async def event_generator():
        last_id = since_id
        while True:
            events = bridge.get_events(since_id=last_id)
            for ev in events:
                last_id = ev["event_id"]
                yield f"data: {json.dumps(ev)}\n\n"
            await asyncio.sleep(0.5)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/")
def index():
    index_path = WEB_DIR / "index.html"
    if not index_path.exists():
        raise HTTPException(status_code=404, detail="UI not built yet.")
    return FileResponse(index_path)


if WEB_DIR.exists():
    app.mount("/web", StaticFiles(directory=str(WEB_DIR)), name="web")


if __name__ == "__main__":
    import uvicorn

    host = os.getenv("HRI_WEB_HOST", "0.0.0.0")
    port = int(os.getenv("HRI_WEB_PORT", "8010"))
    uvicorn.run(app, host=host, port=port)
