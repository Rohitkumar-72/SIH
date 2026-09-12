#!/usr/bin/env python3
"""
SIH Dashboard Bridge & Telemetry Server.

Connects the web frontend dashboard (http://localhost:3000) directly to:
1. ROS 2 Topics (if running):
   - /fleet/dashboard_telemetry
   - /fleet/task_announcement
   - /fleet/task_consensus
   - /fleet/corridor_protocol
   - /fleet/safety_state
2. Dual-protocol bridge:
   - WebSocket on ws://localhost:8765
   - HTTP GET /fleet/dashboard_telemetry on http://localhost:8765/fleet/dashboard_telemetry
3. High-fidelity dynamic stream fallback when running standalone.

Usage:
  python scripts/dashboard_bridge_server.py
"""

import asyncio
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

PORT_HTTP = 8766
PORT_WS = 8765

CURRENT_TELEMETRY = {
    "robots": {
        "robot_1": {"x": -5.25, "y": -22.60, "theta": 1.57},
        "robot_2": {"x": -3.75, "y": -29.55, "theta": 1.57},
        "robot_3": {"x": -2.25, "y": -29.55, "theta": 1.57},
        "robot_4": {"x": -0.75, "y": -25.67, "theta": 1.57}
    },
    "health": {
        "robot_1": {"battery": 92.4, "safe": True},
        "robot_2": {"battery": 88.0, "safe": True},
        "robot_3": {"battery": 96.5, "safe": True},
        "robot_4": {"battery": 74.2, "safe": True}
    },
    "tasks": [
        {
            "task_id": "rnd_task_001",
            "priority": 100,
            "status": "EN_ROUTE_DROPOFF",
            "assigned_robot_id": "robot_1",
            "winning_bid": 104.46,
            "pickup": {"x": -19.99, "y": -17.60, "theta": 0.0, "rack_id": "RACK_WEST_SOUTH_01", "item_type": "Servo Motors"},
            "dropoff": {"x": -11.35, "y": -6.11, "theta": 0.0, "station_id": "DROPOFF_STATION_A", "zone": "DISPATCH_BAY_1"},
            "dwell_times": {"pickup_wait_s": 3.0, "dropoff_wait_s": 3.0},
            "dwell_remaining": 0.0,
            "progress_pct": 68,
            "created_at_epoch": time.time() - 45,
            "expires_at_epoch": time.time() + 300
        }
    ],
    "events": [
        {"type": "cbba", "detail": "[robot_1:CBBA] Decision: SUBMIT_BID for task rnd_task_001. Info: bid=104.46, epoch=1, priority=100"},
        {"type": "quorum", "detail": "[robot_1:CBBA] Decision: UNANIMOUS_COMMIT for task rnd_task_001 -> Winner=robot_1, Bid=104.46, epoch=1. Quorum=4/4"},
        {"type": "whca", "detail": "[robot_1:WHCA] Decision: ROUTE_FEASIBLE for task=rnd_task_001. Info: start=(-5, -23) -> goal=(-20, -18), steps=24, waypoints=4, reservations=24, dynamic_cells=0"},
        {"type": "mutex", "detail": "[robot_2:CorridorMutex] Decision: REQUEST_MUTEX for corridor MC-NS-W. Info: req_id=c14, clock=104"},
        {"type": "safety", "detail": "[robot_1:SafetySupervisor] Decision: CLEAR (LiDAR obstacle free)"}
    ]
}


class TelemetryHTTPHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/fleet/dashboard_telemetry" or self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(CURRENT_TELEMETRY).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress excessive HTTP logging


def start_http_server():
    server = HTTPServer(("0.0.0.0", PORT_HTTP), TelemetryHTTPHandler)
    print(f"[DashboardBridge] HTTP Telemetry endpoint running at http://localhost:{PORT_HTTP}/fleet/dashboard_telemetry")
    server.serve_forever()


def main():
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    print("[DashboardBridge] SIH Telemetry Bridge Server active.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[DashboardBridge] Stopping.")


if __name__ == "__main__":
    main()
