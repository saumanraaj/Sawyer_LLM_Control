#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${HRI_WEB_URL:-http://localhost:8010}"

echo "Health check..."
curl -s "${BASE_URL}/health" | python3 -m json.tool

echo "Scenario A command: move left"
curl -s -X POST "${BASE_URL}/api/command" \
  -H "Content-Type: application/json" \
  -d '{"text":"move left"}' | python3 -m json.tool
sleep 1
curl -s -X POST "${BASE_URL}/api/respond" \
  -H "Content-Type: application/json" \
  -d '{"response":"edit: move left by 10 centimeters"}' | python3 -m json.tool
sleep 1
curl -s -X POST "${BASE_URL}/api/respond" \
  -H "Content-Type: application/json" \
  -d '{"response":"confirm"}' | python3 -m json.tool

echo "Scenario B command: move down by 20 centimeters"
curl -s -X POST "${BASE_URL}/api/command" \
  -H "Content-Type: application/json" \
  -d '{"text":"move down by 20 centimeters"}' | python3 -m json.tool
sleep 1
curl -s -X POST "${BASE_URL}/api/respond" \
  -H "Content-Type: application/json" \
  -d '{"response":"edit: move down by 3 centimeters"}' | python3 -m json.tool
sleep 1
curl -s -X POST "${BASE_URL}/api/respond" \
  -H "Content-Type: application/json" \
  -d '{"response":"confirm"}' | python3 -m json.tool

echo "Current state:"
curl -s "${BASE_URL}/api/state" | python3 -m json.tool
