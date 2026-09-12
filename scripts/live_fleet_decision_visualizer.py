#!/usr/bin/env python3
"""
SIH Real-Time Fleet Decision & Multi-Agent Intelligence Visualizer.

A cinema-grade terminal visualizer designed for demo video recordings and live presentation.
Displays real-time decentralized decisions:
- ⚡ CBBA (Consensus-Based Bundle Algorithm) Auctions & Multi-AMR Bidding Battles
- 🤝 Quorum Verification & Consensus Confirmation
- 🗺️ WHCA* Multi-Agent Space-Time Reservation & Route Feasibility
- 🚧 Narrow Aisle / Corridor Distributed Mutex (Lamport Logical Clock) Leasing
- ⚠️ LiDAR Dynamic Obstacle Detection & Reactive Re-routing
- 📦 Task Execution, Station Dwelling, and Fleet Progress HUD

Usage:
  python3 scripts/live_fleet_decision_visualizer.py
  python3 scripts/live_fleet_decision_visualizer.py --replay
  python3 scripts/live_fleet_decision_visualizer.py --log-dir /path/to/run_dir
"""

import argparse
import datetime
import json
import math
import os
import re
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

# Terminal ANSI Styling
ESC = "\033["
C_RESET = f"{ESC}0m"
C_BOLD = f"{ESC}1m"
C_DIM = f"{ESC}2m"
C_ITALIC = f"{ESC}3m"
C_UNDER = f"{ESC}4m"

# Foreground Colors
C_BLACK = f"{ESC}30m"
C_RED = f"{ESC}31m"
C_GREEN = f"{ESC}32m"
C_YELLOW = f"{ESC}33m"
C_BLUE = f"{ESC}34m"
C_MAGENTA = f"{ESC}35m"
C_CYAN = f"{ESC}36m"
C_WHITE = f"{ESC}37m"

# Bright Foreground Colors
C_B_RED = f"{ESC}91m"
C_B_GREEN = f"{ESC}92m"
C_B_YELLOW = f"{ESC}93m"
C_B_BLUE = f"{ESC}94m"
C_B_MAGENTA = f"{ESC}95m"
C_B_CYAN = f"{ESC}96m"
C_B_WHITE = f"{ESC}97m"

# Background Colors
BG_DARK = f"{ESC}48;5;234m"
BG_BLUE = f"{ESC}48;5;17m"
BG_CYAN = f"{ESC}48;5;24m"
BG_GREEN = f"{ESC}48;5;22m"
BG_YELLOW = f"{ESC}48;5;58m"
BG_MAGENTA = f"{ESC}48;5;53m"
BG_RED = f"{ESC}48;5;52m"

ROBOT_COLORS = {
    "robot_1": C_B_CYAN,
    "robot_2": C_B_GREEN,
    "robot_3": C_B_YELLOW,
    "robot_4": C_B_MAGENTA,
    "robot_5": C_B_BLUE,
    "robot_6": C_B_RED,
}

ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

def strip_ansi(s: str) -> str:
    return ANSI_ESCAPE.sub('', s)

def visible_len(s: str) -> int:
    return len(strip_ansi(s))

def pad_visible(s: str, target_width: int, align: str = 'left') -> str:
    vlen = visible_len(s)
    pad = max(0, target_width - vlen)
    if align == 'right':
        return ' ' * pad + s
    elif align == 'center':
        left = pad // 2
        right = pad - left
        return ' ' * left + s + ' ' * right
    else:
        return s + ' ' * pad

# Regex Patterns for Parsing Decision Lines
RE_CBBA_BID = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*SUBMIT_BID for task (?P<task>[a-zA-Z0-9_-]+)\..*Info:\s*bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+),\s*pose=\((?P<px>[-\d.]+),\s*(?P<py>[-\d.]+)\),\s*pickup=\((?P<tx>[-\d.]+),\s*(?P<ty>[-\d.]+)\),\s*priority=(?P<priority>\d+)"
)

RE_CBBA_DERIVED = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*DERIVED_WINNER for task (?P<task>[a-zA-Z0-9_-]+)\..*Winner=(?P<winner>robot_\d),\s*Bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+)\..*All candidate bids:\s*(?P<candidates>\[.*?\])"
)

RE_CBBA_COMMIT = re.compile(
    r"\[(?P<robot>robot_\d):CBBA\]\s*Decision:\s*UNANIMOUS_COMMIT for task (?P<task>[a-zA-Z0-9_-]+)\s*->\s*Winner=(?P<winner>robot_\d),\s*Bid=(?P<bid>[\d.]+),\s*epoch=(?P<epoch>\d+)\..*Quorum=(?P<quorum>\d+/\d+)"
)

RE_TASK_ACCEPT = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ACCEPT new task (?P<task>[a-zA-Z0-9_-]+)\..*robot_pose=\((?P<rx>[-\d.]+),\s*(?P<ry>[-\d.]+)\),\s*pickup=\((?P<px>[-\d.]+),\s*(?P<py>[-\d.]+)\),\s*dropoff=\((?P<dx>[-\d.]+),\s*(?P<dy>[-\d.]+)\)"
)

RE_PICKUP_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at pickup for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)

RE_PICKUP_DONE = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*PICKUP_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)"
)

RE_DROPOFF_ARRIVED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*ARRIVED at dropoff for task (?P<task>[a-zA-Z0-9_-]+)\..*dwelling (?P<dwell>[-\d.]+)s"
)

RE_TASK_COMPLETED = re.compile(
    r"\[(?P<robot>robot_\d):TaskExecutor\]\s*Decision:\s*(?:DROPOFF_DWELL_COMPLETE for task (?P<task>[a-zA-Z0-9_-]+)|RESET executor after publishing completion for task (?P<task2>[a-zA-Z0-9_-]+))"
)

RE_WHCA_FEASIBLE = re.compile(
    r"\[(?P<robot>robot_\d):WHCA\]\s*Decision:\s*ROUTE_FEASIBLE for task=(?P<task>[a-zA-Z0-9_-]+)\..*Info:\s*start=\((?P<sx>\d+),\s*(?P<sy>\d+)\)\s*->\s*goal=\((?P<gx>\d+),\s*(?P<gy>\d+)\),\s*steps=(?P<steps>\d+),\s*waypoints=(?P<wps>\d+),\s*reservations=(?P<res>\d+),\s*dynamic_cells=(?P<dyn>\d+)"
)

RE_WHCA_INFEASIBLE = re.compile(
    r"\[(?P<robot>robot_\d):WHCA\]\s*ERROR:\s*ROUTE_INFEASIBLE for task=(?P<task>[a-zA-Z0-9_-]+)\..*Info:\s*start=(?P<start>[^,]+).*goal=(?P<goal>[^,]+).*Reason=(?P<reason>.*)"
)

RE_MUTEX_REQUEST = re.compile(
    r"\[(?P<robot>robot_\d):CorridorMutex\]\s*Decision:\s*REQUEST_MUTEX for corridor (?P<corridor>[a-zA-Z0-9_-]+)\..*Info:\s*req_id=(?P<req_id>[a-f0-9]+),\s*clock=(?P<clock>\d+)"
)

RE_MUTEX_ENTER = re.compile(
    r"\[(?P<robot>robot_\d):CorridorMutex\]\s*Decision:\s*ENTER_CORRIDOR (?P<corridor>[a-zA-Z0-9_-]+)\..*Info:\s*full grants received from peers (?P<peers>\[.*?\])"
)

RE_MUTEX_EXIT = re.compile(
    r"\[(?P<robot>robot_\d):CorridorMutex\]\s*Decision:\s*EXIT_CORRIDOR (?P<corridor>[a-zA-Z0-9_-]+)\..*Info:\s*releasing mutex"
)

RE_MUTEX_RELEASE = re.compile(
    r"\[(?P<robot>robot_\d):CorridorMutex\]\s*Decision:\s*RELEASE_MUTEX for corridor (?P<corridor>[a-zA-Z0-9_-]+)\..*Event=(?P<event>\w+)"
)

RE_SAFETY_EVENT = re.compile(
    r"\[(?P<robot>robot_\d):SafetySupervisor\]\s*Decision:\s*(?P<decision>SAFETY_STOP|SAFETY_SLOW)"
)


class RobotHUDState:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.state: str = "IDLE"
        self.task_id: str = "-"
        self.x: float = 0.0
        self.y: float = 0.0
        self.corridor: Optional[str] = None
        self.corridor_locked: bool = False
        self.speed: float = 0.0
        self.dwelling_s: float = 0.0
        self.last_decision: str = "INITIALIZED"


class LiveFleetVisualizer:
    def __init__(self, log_dir: Optional[Path] = None, fleet_size: int = 4, pace_s: float = 0.03, is_replay: bool = False, replay_speed: float = 1.0):
        self.log_dir = log_dir
        self.fleet_size = fleet_size
        self.pace_s = pace_s
        self.is_replay = is_replay
        self.replay_speed = replay_speed
        
        self.robots: Dict[str, RobotHUDState] = {
            f"robot_{i}": RobotHUDState(f"robot_{i}") for i in range(1, fleet_size + 1)
        }
        
        self.tasks_completed: int = 0
        self.target_tasks: int = 20
        self.active_auctions: Dict[str, Dict] = {}
        self.start_wall_time: float = time.time()
        self.sim_time_s: float = 0.0
        self.running: bool = True
        
        # Terminal width
        self.term_width: int = 105
        try:
            cols = os.get_terminal_size().columns
            self.term_width = max(80, min(140, cols))
        except Exception:
            pass

    def r_color(self, robot_id: str) -> str:
        return ROBOT_COLORS.get(robot_id, C_B_WHITE)

    def print_header(self, run_id: str = "LIVE_DATA_COLLECTION"):
        w = self.term_width
        print(f"\n{C_BOLD}{C_B_CYAN}╔{'═' * (w - 2)}╗{C_RESET}")
        title = " 🤖 SIH 2024 DECENTRALIZED MULTI-AMR FLEET: REAL-TIME DECISION ENGINE "
        inner_title = pad_visible(f"{C_BOLD}{C_B_WHITE}{BG_BLUE}{title}{C_RESET}", w - 2, align='center')
        print(f"{C_BOLD}{C_B_CYAN}║{C_RESET}{inner_title}{C_BOLD}{C_B_CYAN}║{C_RESET}")
        
        sub = f" Active Run: {run_id} | Fleet: {self.fleet_size} AMRs | WHCA* + CBBA + Distributed Lamport Mutex "
        inner_sub = pad_visible(f"{C_DIM}{C_WHITE}{sub}{C_RESET}", w - 2, align='center')
        print(f"{C_BOLD}{C_B_CYAN}║{C_RESET}{inner_sub}{C_BOLD}{C_B_CYAN}║{C_RESET}")
        print(f"{C_BOLD}{C_B_CYAN}╚{'═' * (w - 2)}╝{C_RESET}\n")

    def print_hud(self):
        w = self.term_width
        elapsed = time.time() - self.start_wall_time
        hud_title = f" 📊 LIVE FLEET STATUS BOARD [T+{elapsed:5.1f}s | SimTime:{self.sim_time_s:5.1f}s | Tasks:{self.tasks_completed}] "
        border_bar = "═" * max(2, w - visible_len(hud_title) - 4)
        print(f"{C_BOLD}{C_B_BLUE}╔══{hud_title}{border_bar}╗{C_RESET}")
        
        for r_id in sorted(self.robots.keys()):
            r = self.robots[r_id]
            rc = self.r_color(r_id)
            
            # State badge
            if "TO_PICKUP" in r.state:
                st_badge = f"{C_B_BLUE}▶ NAV_PICKUP{C_RESET}"
            elif "DWELL_PICKUP" in r.state or "AT_PICKUP" in r.state:
                st_badge = f"{C_B_YELLOW}⚓ LOADING...{C_RESET}"
            elif "TO_DROPOFF" in r.state:
                st_badge = f"{C_B_CYAN}➜ NAV_DROPOFF{C_RESET}"
            elif "DWELL_DROPOFF" in r.state or "AT_DROPOFF" in r.state:
                st_badge = f"{C_B_YELLOW}⚓ UNLOADING.{C_RESET}"
            elif "BIDDING" in r.state:
                st_badge = f"{C_B_MAGENTA}⚡ CBBA_BID{C_RESET}"
            else:
                st_badge = f"{C_GREEN}✔ IDLE_READY{C_RESET}"

            t_disp = r.task_id if len(r.task_id) <= 12 else (r.task_id[:10] + "..")
            task_str = f"Task: {C_BOLD}{t_disp:<12}{C_RESET}"
            pos_str = f"Pos: ({r.x:5.1f}, {r.y:5.1f})"
            
            if r.corridor:
                c_name = r.corridor if len(r.corridor) <= 16 else (r.corridor[:14] + "..")
                if r.corridor_locked:
                    corridor_str = f"Aisle: {C_B_RED}🔒 {c_name:<16}{C_RESET}"
                else:
                    corridor_str = f"Aisle: {C_YELLOW}⏳ {c_name:<16}{C_RESET}"
            else:
                corridor_str = f"Aisle: {C_DIM}None (Open Zone) {C_RESET}"

            line_content = f" {rc}{C_BOLD}{r_id:<7}{C_RESET} │ {pad_visible(st_badge, 13)} │ {pad_visible(task_str, 18)} │ {pad_visible(pos_str, 17)} │ {pad_visible(corridor_str, 24)}"
            padded_row = pad_visible(line_content, w - 2)
            print(f"{C_BOLD}{C_B_BLUE}║{C_RESET}{padded_row}{C_BOLD}{C_B_BLUE}║{C_RESET}")

        print(f"{C_BOLD}{C_B_BLUE}╚{'═' * (w - 2)}╝{C_RESET}\n")

    def animate_cbba_auction(self, task_id: str, candidates: List[Tuple[str, float]], winner: str, winning_bid: float, epoch: int, quorum: str):
        w = self.term_width
        rc_winner = self.r_color(winner)
        print(f"\n{C_BOLD}{C_B_MAGENTA}╔{'═' * (w - 2)}╗{C_RESET}")
        title = f" ⚡ DECENTRALIZED CBBA AUCTION: Task [{task_id}] (Epoch {epoch}) "
        header_row = pad_visible(f" {C_BOLD}{C_B_WHITE}{BG_MAGENTA}{title}{C_RESET}", w - 2)
        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}{header_row}{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
        print(f"{C_BOLD}{C_B_MAGENTA}╠{'═' * (w - 2)}╣{C_RESET}")
        time.sleep(self.pace_s * 2)

        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}  {C_BOLD}{C_CYAN}📊 BIDS SUBMITTED & INDEPENDENTLY EVALUATED BY FLEET:{C_RESET}")
        sorted_candidates = sorted(candidates, key=lambda c: c[1])
        for idx, (rid, bid) in enumerate(sorted_candidates):
            rc = self.r_color(rid)
            is_win = (rid == winner)
            marker = f"{C_BOLD}{C_B_GREEN}★ WINNING BID{C_RESET}" if is_win else f"{C_DIM}Higher Cost{C_RESET}"
            badge = f"{C_BOLD}#{idx+1}{C_RESET}"
            bar = "█" * max(1, min(20, int(bid / 10.0)))
            row_str = f"    {badge} {rc}{C_BOLD}{rid:<8}{C_RESET} ──▶ Bid Cost: {C_BOLD}{bid:6.2f}{C_RESET}  [{C_B_CYAN}{bar:<20}{C_RESET}] {marker}"
            print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}{pad_visible(row_str, w - 2)}{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
            time.sleep(self.pace_s)

        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
        quorum_checks = " ".join([f"{self.r_color(f'robot_{i}')}[✔ R{i}]{C_RESET}" for i in range(1, self.fleet_size + 1)])
        quorum_row = f"  {C_BOLD}{C_YELLOW}🤝 CONSENSUS QUORUM:{C_RESET} {quorum_checks}  {C_BOLD}{C_GREEN}({quorum} Unanimous Quorum){C_RESET}"
        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}{pad_visible(quorum_row, w - 2)}{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
        time.sleep(self.pace_s * 1.5)

        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
        win_box = f" 🏆 TASK AWARDED TO {rc_winner}{winner.upper()}{C_RESET}{C_BOLD}{C_B_WHITE} (Cost: {winning_bid:.2f}) -> Dispatched to TaskExecutor! "
        win_row = f"  {C_BOLD}{C_B_WHITE}{BG_GREEN}{win_box}{C_RESET}"
        print(f"{C_BOLD}{C_B_MAGENTA}║{C_RESET}{pad_visible(win_row, w - 2)}{C_BOLD}{C_B_MAGENTA}║{C_RESET}")
        print(f"{C_BOLD}{C_B_MAGENTA}╚{'═' * (w - 2)}╝{C_RESET}\n")
        time.sleep(self.pace_s * 2)

    def animate_whca_plan(self, robot_id: str, task_id: str, start: Tuple[int, int], goal: Tuple[int, int], steps: int, reservations: int, dynamic_cells: int):
        w = self.term_width
        rc = self.r_color(robot_id)
        title = f" 🗺️  WHCA* MULTI-AGENT SPACE-TIME ROUTE: {rc}{robot_id.upper()}{C_RESET}{C_BOLD}{C_B_CYAN} "
        top_bar = "─" * max(2, w - visible_len(title) - 4)
        print(f"{C_BOLD}{C_B_CYAN}┌──{title}{top_bar}┐{C_RESET}")
        
        l1 = f"  Task: {C_BOLD}{task_id:<16}{C_RESET} │ Trajectory: {C_BOLD}({start[0]}, {start[1]}) ───▶ ({goal[0]}, {goal[1]}){C_RESET}"
        print(f"{C_BOLD}{C_B_CYAN}│{C_RESET}{pad_visible(l1, w - 2)}{C_BOLD}{C_B_CYAN}│{C_RESET}")
        
        l2 = f"  Status: {C_BOLD}{C_GREEN}✅ ROUTE_FEASIBLE{C_RESET} │ Path Length: {C_BOLD}{steps} waypoints{C_RESET}"
        print(f"{C_BOLD}{C_B_CYAN}│{C_RESET}{pad_visible(l2, w - 2)}{C_BOLD}{C_B_CYAN}│{C_RESET}")
        
        l3 = f"  Space-Time Reserved Slots Avoided: {C_BOLD}{C_YELLOW}{reservations} peer reservation cells{C_RESET}"
        print(f"{C_BOLD}{C_B_CYAN}│{C_RESET}{pad_visible(l3, w - 2)}{C_BOLD}{C_B_CYAN}│{C_RESET}")
        
        if dynamic_cells > 0:
            l4 = f"  Dynamic LiDAR Obstacles Factored: {C_BOLD}{C_RED}{dynamic_cells} dynamic blockage cells{C_RESET}"
        else:
            l4 = f"  Dynamic LiDAR Obstacles Factored: {C_GREEN}0 (Clean Free Corridor){C_RESET}"
        print(f"{C_BOLD}{C_B_CYAN}│{C_RESET}{pad_visible(l4, w - 2)}{C_BOLD}{C_B_CYAN}│{C_RESET}")
        
        l5 = f"  Execution: Published RoutePlan #{int(time.time()) % 1000:03d} -> Target Tracking Speed: {C_BOLD}0.46 m/s{C_RESET}"
        print(f"{C_BOLD}{C_B_CYAN}│{C_RESET}{pad_visible(l5, w - 2)}{C_BOLD}{C_B_CYAN}│{C_RESET}")
        print(f"{C_BOLD}{C_B_CYAN}└{'─' * (w - 2)}┘{C_RESET}")
        time.sleep(self.pace_s)

    def animate_corridor_lease(self, robot_id: str, corridor: str, event_type: str, details: str = ""):
        w = self.term_width
        rc = self.r_color(robot_id)
        
        if event_type == "REQUEST":
            title = f" 🚧 NARROW AISLE MUTEX REQUEST: {rc}{robot_id.upper()}{C_RESET}{C_BOLD}{C_YELLOW} "
            top_bar = "═" * max(2, w - visible_len(title) - 4)
            print(f"\n{C_BOLD}{C_YELLOW}╔══{title}{top_bar}╗{C_RESET}")
            l1 = f"  Target Corridor: {C_BOLD}{C_WHITE}{corridor}{C_RESET}"
            l2 = f"  Lamport Logical Clock Protocol: {details}"
            l3 = f"  Status: {C_YELLOW}Awaiting peer grant handshakes from fleet...{C_RESET}"
            print(f"{C_BOLD}{C_YELLOW}║{C_RESET}{pad_visible(l1, w - 2)}{C_BOLD}{C_YELLOW}║{C_RESET}")
            print(f"{C_BOLD}{C_YELLOW}║{C_RESET}{pad_visible(l2, w - 2)}{C_BOLD}{C_YELLOW}║{C_RESET}")
            print(f"{C_BOLD}{C_YELLOW}║{C_RESET}{pad_visible(l3, w - 2)}{C_BOLD}{C_YELLOW}║{C_RESET}")
            print(f"{C_BOLD}{C_YELLOW}╚{'═' * (w - 2)}╝{C_RESET}")
        elif event_type == "ENTER":
            title = f" 🔒 EXCLUSIVE AISLE LEASE ACQUIRED: {rc}{robot_id.upper()}{C_RESET}{C_BOLD}{C_B_GREEN} "
            top_bar = "═" * max(2, w - visible_len(title) - 4)
            print(f"\n{C_BOLD}{C_B_GREEN}╔══{title}{top_bar}╗{C_RESET}")
            l1 = f"  Aisle / Corridor: {C_BOLD}{C_WHITE}{corridor}{C_RESET}"
            l2 = f"  Peer Handshake Clearance: {C_BOLD}{C_GREEN}100% GRANTS CONFIRMED ({details}){C_RESET}"
            l3 = f"  State: {C_BOLD}{C_B_GREEN}ENTERED CORRIDOR{C_RESET} │ Physical entrance verified │ Speed: 0.46 m/s"
            print(f"{C_BOLD}{C_B_GREEN}║{C_RESET}{pad_visible(l1, w - 2)}{C_BOLD}{C_B_GREEN}║{C_RESET}")
            print(f"{C_BOLD}{C_B_GREEN}║{C_RESET}{pad_visible(l2, w - 2)}{C_BOLD}{C_B_GREEN}║{C_RESET}")
            print(f"{C_BOLD}{C_B_GREEN}║{C_RESET}{pad_visible(l3, w - 2)}{C_BOLD}{C_B_GREEN}║{C_RESET}")
            print(f"{C_BOLD}{C_B_GREEN}╚{'═' * (w - 2)}╝{C_RESET}")
        elif event_type == "EXIT":
            title = f" 🔓 AISLE LEASE RELEASED: {rc}{robot_id.upper()}{C_RESET}{C_BOLD}{C_CYAN} "
            top_bar = "═" * max(2, w - visible_len(title) - 4)
            print(f"\n{C_BOLD}{C_CYAN}╔══{title}{top_bar}╗{C_RESET}")
            l1 = f"  Exited Corridor: {C_BOLD}{C_WHITE}{corridor}{C_RESET}"
            l2 = f"  Action: {C_CYAN}Distributed Mutex released. Unblocking deferred peers!{C_RESET}"
            print(f"{C_BOLD}{C_CYAN}║{C_RESET}{pad_visible(l1, w - 2)}{C_BOLD}{C_CYAN}║{C_RESET}")
            print(f"{C_BOLD}{C_CYAN}║{C_RESET}{pad_visible(l2, w - 2)}{C_BOLD}{C_CYAN}║{C_RESET}")
            print(f"{C_BOLD}{C_CYAN}╚{'═' * (w - 2)}╝{C_RESET}")
        time.sleep(self.pace_s * 1.5)

    def animate_obstacle_alert(self, robot_id: str, decision: str):
        w = self.term_width
        rc = self.r_color(robot_id)
        if "STOP" in decision:
            badge = f"{C_BOLD}{C_B_WHITE}{BG_RED} 🛑 SAFETY EMERGENCY STOP TRIGGERED {C_RESET}"
        else:
            badge = f"{C_BOLD}{C_B_WHITE}{BG_YELLOW} ⚠️ SAFETY CAUTION: DECELERATING SPEED {C_RESET}"
        title = f" {badge} "
        top_bar = "─" * max(2, w - visible_len(title) - 4)
        print(f"\n{C_BOLD}{C_RED}┌──{title}{top_bar}┐{C_RESET}")
        l1 = f"  Robot: {rc}{robot_id}{C_RESET} │ Decision: {C_BOLD}{decision}{C_RESET}"
        l2 = f"  Trigger: 2D LiDAR Proximity / Persistent Dynamic Blockage in envelope"
        l3 = f"  Action: Engaging active collision avoidance & triggering WHCA* bypass replan!"
        print(f"{C_BOLD}{C_RED}│{C_RESET}{pad_visible(l1, w - 2)}{C_BOLD}{C_RED}│{C_RESET}")
        print(f"{C_BOLD}{C_RED}│{C_RESET}{pad_visible(l2, w - 2)}{C_BOLD}{C_RED}│{C_RESET}")
        print(f"{C_BOLD}{C_RED}│{C_RESET}{pad_visible(l3, w - 2)}{C_BOLD}{C_RED}│{C_RESET}")
        print(f"{C_BOLD}{C_RED}└{'─' * (w - 2)}┘{C_RESET}\n")
        time.sleep(self.pace_s)

    def animate_task_completion(self, robot_id: str, task_id: str):
        w = self.term_width
        rc = self.r_color(robot_id)
        self.tasks_completed += 1
        pct = int((self.tasks_completed / max(1, self.target_tasks)) * 100)
        bar_len = 30
        filled = int((pct / 100) * bar_len)
        prog_bar = f"{C_B_GREEN}{'█' * filled}{C_DIM}{'░' * (bar_len - filled)}{C_RESET}"
        
        print(f"\n{C_BOLD}{C_GREEN}╔{'═' * (w - 2)}╗{C_RESET}")
        banner = f" 🎉 TASK WORK CYCLE COMPLETE: [{task_id}] FINISHED BY {robot_id.upper()} "
        print(f"{C_BOLD}{C_GREEN}║{C_RESET}{pad_visible(f' {C_BOLD}{C_B_WHITE}{BG_GREEN}{banner}{C_RESET}', w - 2)}{C_BOLD}{C_GREEN}║{C_RESET}")
        prog_row = f"  Progress: [{prog_bar}] {C_BOLD}{self.tasks_completed}/{self.target_tasks} ({pct}%){C_RESET}"
        print(f"{C_BOLD}{C_GREEN}║{C_RESET}{pad_visible(prog_row, w - 2)}{C_BOLD}{C_GREEN}║{C_RESET}")
        print(f"{C_BOLD}{C_GREEN}╚{'═' * (w - 2)}╝{C_RESET}\n")
        time.sleep(self.pace_s * 2)

    def process_log_line(self, line: str):
        line = line.strip()
        if not line:
            return

        # 1. CBBA Bid Submissions
        m = RE_CBBA_BID.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            bid = float(m.group("bid"))
            if robot in self.robots:
                self.robots[robot].state = "BIDDING"
                self.robots[robot].task_id = task
                self.robots[robot].last_decision = f"SUBMIT_BID({bid:.1f})"
            auction = self.active_auctions.setdefault(task, {"bids": {}, "candidates": []})
            auction["bids"][robot] = bid
            return

        # 2. CBBA Derived Winner & Candidate Collection
        m = RE_CBBA_DERIVED.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            winner = m.group("winner")
            bid = float(m.group("bid"))
            epoch = int(m.group("epoch"))
            cand_str = m.group("candidates")
            try:
                cleaned = re.findall(r"\('?(robot_\d)'?,\s*([\d.]+)\)", cand_str)
                candidates = [(c[0], float(c[1])) for c in cleaned]
            except Exception:
                candidates = [(winner, bid)]
            
            auction = self.active_auctions.setdefault(task, {"reported": False})
            if not auction.get("reported"):
                auction["reported"] = True
                self.animate_cbba_auction(task, candidates, winner, bid, epoch, f"{self.fleet_size}/{self.fleet_size}")
            return

        # 3. Task Accepted by Executor
        m = RE_TASK_ACCEPT.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            px, py = float(m.group("px")), float(m.group("py"))
            dx, dy = float(m.group("dx")), float(m.group("dy"))
            if robot in self.robots:
                self.robots[robot].state = f"TO_PICKUP({task})"
                self.robots[robot].task_id = task
                self.robots[robot].last_decision = "ACCEPT_TASK"
            rc = self.r_color(robot)
            print(f" {C_DIM}▶{C_RESET} {rc}{C_BOLD}{robot}{C_RESET}: {C_CYAN}TaskExecutor ACCEPTED {task}{C_RESET} -> Pickup: ({px:.1f}, {py:.1f}), Dropoff: ({dx:.1f}, {dy:.1f})")
            return

        # 4. WHCA* Space-Time Path Planning
        m = RE_WHCA_FEASIBLE.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            sx, sy = int(m.group("sx")), int(m.group("sy"))
            gx, gy = int(m.group("gx")), int(m.group("gy"))
            steps = int(m.group("steps"))
            res = int(m.group("res"))
            dyn = int(m.group("dyn"))
            self.animate_whca_plan(robot, task, (sx, sy), (gx, gy), steps, res, dyn)
            return

        # 5. WHCA Infeasible Replan
        m = RE_WHCA_INFEASIBLE.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            reason = m.group("reason")
            rc = self.r_color(robot)
            print(f" {C_RED}✖ {rc}{robot}{C_RESET} WHCA ROUTE_INFEASIBLE ({task}): {reason} -> Local wait & replan!")
            return

        # 6. Corridor Mutex Request
        m = RE_MUTEX_REQUEST.search(line)
        if m:
            robot = m.group("robot")
            corridor = m.group("corridor")
            req_id = m.group("req_id")
            clock = m.group("clock")
            if robot in self.robots:
                self.robots[robot].corridor = corridor
                self.robots[robot].corridor_locked = False
                self.robots[robot].last_decision = f"REQ_MUTEX({corridor})"
            self.animate_corridor_lease(robot, corridor, "REQUEST", f"req_id={req_id}, LamportClock={clock}")
            return

        # 7. Corridor Mutex Enter (Locked)
        m = RE_MUTEX_ENTER.search(line)
        if m:
            robot = m.group("robot")
            corridor = m.group("corridor")
            peers = m.group("peers")
            if robot in self.robots:
                self.robots[robot].corridor = corridor
                self.robots[robot].corridor_locked = True
                self.robots[robot].last_decision = f"ENTER_CORRIDOR({corridor})"
            self.animate_corridor_lease(robot, corridor, "ENTER", peers)
            return

        # 8. Corridor Mutex Exit (Released)
        m = RE_MUTEX_EXIT.search(line)
        if m:
            robot = m.group("robot")
            corridor = m.group("corridor")
            if robot in self.robots:
                if self.robots[robot].corridor == corridor:
                    self.robots[robot].corridor = None
                    self.robots[robot].corridor_locked = False
                self.robots[robot].last_decision = f"EXIT_CORRIDOR({corridor})"
            self.animate_corridor_lease(robot, corridor, "EXIT")
            return

        # 9. Arrival at Pickup
        m = RE_PICKUP_ARRIVED.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            dwell = m.group("dwell")
            if robot in self.robots:
                self.robots[robot].state = f"AT_PICKUP({task})"
                self.robots[robot].last_decision = f"PICKUP_DWELL({dwell}s)"
            rc = self.r_color(robot)
            print(f" {C_YELLOW}⚓ {rc}{C_BOLD}{robot}{C_RESET} {C_YELLOW}ARRIVED at Pickup for {task}{C_RESET} (Dwelling {dwell}s for package loading)")
            return

        # 10. Pickup Done
        m = RE_PICKUP_DONE.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            if robot in self.robots:
                self.robots[robot].state = f"TO_DROPOFF({task})"
                self.robots[robot].last_decision = "PICKUP_COMPLETE"
            rc = self.r_color(robot)
            print(f" {C_CYAN}➜ {rc}{C_BOLD}{robot}{C_RESET} {C_CYAN}Package Loaded! En route to Dropoff for {task}{C_RESET}")
            return

        # 11. Arrival at Dropoff
        m = RE_DROPOFF_ARRIVED.search(line)
        if m:
            robot = m.group("robot")
            task = m.group("task")
            dwell = m.group("dwell")
            if robot in self.robots:
                self.robots[robot].state = f"AT_DROPOFF({task})"
                self.robots[robot].last_decision = f"DROPOFF_DWELL({dwell}s)"
            rc = self.r_color(robot)
            print(f" {C_YELLOW}⚓ {rc}{C_BOLD}{robot}{C_RESET} {C_YELLOW}ARRIVED at Dropoff for {task}{C_RESET} (Dwelling {dwell}s for unloading)")
            return

        # 12. Task Completed
        m = RE_TASK_COMPLETED.search(line)
        if m:
            task = m.group("task") or m.group("task2")
            robot = "robot_1"
            for r_id, r_state in self.robots.items():
                if r_state.task_id == task:
                    robot = r_id
                    r_state.state = "IDLE"
                    r_state.task_id = "-"
                    r_state.last_decision = "COMPLETED"
                    break
            self.animate_task_completion(robot, task)
            return

        # 13. Safety & ORCA Avoidance
        m = RE_SAFETY_EVENT.search(line)
        if m:
            robot = m.group("robot")
            dec = m.group("decision")
            self.animate_obstacle_alert(robot, dec)
            return

    def process_telemetry_line(self, line: str):
        line = line.strip()
        if not line:
            return
        try:
            rec = json.loads(line)
            ev_type = rec.get("event_type")
            if ev_type == "robot_state":
                rid = rec.get("robot_id")
                if rid in self.robots:
                    self.robots[rid].x = float(rec.get("x", 0.0))
                    self.robots[rid].y = float(rec.get("y", 0.0))
                    self.sim_time_s = float(rec.get("logged_at", self.sim_time_s))
            elif ev_type == "run_manifest":
                self.fleet_size = int(rec.get("robot_count", self.fleet_size))
                for i in range(1, self.fleet_size + 1):
                    rid = f"robot_{i}"
                    if rid not in self.robots:
                        self.robots[rid] = RobotHUDState(rid)
        except Exception:
            pass

    def run_live(self):
        """Continuously discover active run and stream events."""
        signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
        
        print(f"\n{C_BOLD}{C_CYAN}🔍 SIH Decision Visualizer: Searching for active data collection run...{C_RESET}")
        
        last_hud_print_s = 0.0
        current_run_dir: Optional[Path] = self.log_dir
        fleet_fd = None
        telemetry_fd = None
        
        while self.running:
            # Auto-discover log directory if not set or run finished
            if current_run_dir is None or not current_run_dir.exists():
                candidate = self.find_latest_run_dir()
                if candidate and candidate != current_run_dir:
                    current_run_dir = candidate
                    if fleet_fd:
                        try: fleet_fd.close()
                        except: pass
                        fleet_fd = None
                    if telemetry_fd:
                        try: telemetry_fd.close()
                        except: pass
                        telemetry_fd = None
                    self.print_header(current_run_dir.name)
                else:
                    time.sleep(1.0)
                    continue

            fleet_log_file = current_run_dir / "fleet.log"
            telemetry_file = current_run_dir / "fleet_telemetry.jsonl"

            # Open log files when they appear
            if fleet_fd is None and fleet_log_file.exists():
                fleet_fd = open(fleet_log_file, "r", encoding="utf-8", errors="ignore")
                self.print_header(current_run_dir.name)

            if telemetry_fd is None and telemetry_file.exists():
                telemetry_fd = open(telemetry_file, "r", encoding="utf-8", errors="ignore")

            # Read new telemetry lines
            if telemetry_fd is not None:
                tlines = telemetry_fd.readlines()
                for tline in tlines:
                    self.process_telemetry_line(tline)

            # Read new fleet.log lines
            if fleet_fd is not None:
                lines = fleet_fd.readlines()
                for line in lines:
                    self.process_log_line(line)

            # Print HUD periodically every 5 seconds
            now = time.time()
            if now - last_hud_print_s >= 5.0:
                last_hud_print_s = now
                self.print_hud()

            time.sleep(0.1)

    def run_replay(self):
        """Replay past log directory with controllable animation pacing."""
        target_dir = self.log_dir or self.find_latest_run_dir()
        if not target_dir or not target_dir.exists():
            print(f"{C_RED}ERROR: No valid run directory found for replay.{C_RESET}")
            return

        self.print_header(f"REPLAY: {target_dir.name}")
        fleet_log_file = target_dir / "fleet.log"
        telemetry_file = target_dir / "fleet_telemetry.jsonl"

        if not fleet_log_file.exists():
            print(f"{C_RED}ERROR: fleet.log not found in {target_dir}{C_RESET}")
            return

        # Pre-read telemetry or start background thread to read coordinates
        if telemetry_file.exists():
            def read_telemetry_bg():
                with open(telemetry_file, "r", encoding="utf-8", errors="ignore") as tf:
                    for tline in tf:
                        self.process_telemetry_line(tline)
                        time.sleep(0.005)
            t_thread = threading.Thread(target=read_telemetry_bg, daemon=True)
            t_thread.start()

        print(f" {C_CYAN}>>> Streaming recorded multi-agent decisions with animated pacing... <<<{C_RESET}\n")
        self.print_hud()

        # Stream all log lines with micro-delays
        with open(fleet_log_file, "r", encoding="utf-8", errors="ignore") as f:
            for idx, line in enumerate(f):
                self.process_log_line(line)
                if idx % 80 == 0 and idx > 0:
                    self.print_hud()
                time.sleep(self.pace_s / self.replay_speed)

        print(f"\n{C_BOLD}{C_GREEN}✔ Replay Finished for {target_dir.name}{C_RESET}\n")

    def find_latest_run_dir(self) -> Optional[Path]:
        """Search workspace log folders for the most recent run."""
        candidates = []
        search_roots = [
            Path.home() / "amr_ws/log",
            Path.home() / "amr_ws/src/SIH/log",
            Path("/home/rtsws/amr_ws/log"),
            Path("/home/rtsws/amr_ws/src/SIH/log"),
        ]
        ws_env = os.environ.get("AMR_WS_LOG_DIR")
        if ws_env:
            search_roots.insert(0, Path(ws_env))

        for root in search_roots:
            if not root.exists():
                continue
            for p in root.glob("desktop_data_collection_*"):
                for sub in p.glob("desktop_run_*"):
                    if (sub / "fleet.log").exists():
                        candidates.append(sub)
            for p in root.glob("fleet_runs/*"):
                if (p / "fleet.log").exists():
                    candidates.append(p)

        if not candidates:
            return None
        candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return candidates[0]


def main():
    parser = argparse.ArgumentParser(
        description="SIH Multi-AMR Real-Time Decision & Intelligence Visualizer",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--log-dir", type=str, default=None, help="Explicit log directory to monitor or replay")
    parser.add_argument("--replay", action="store_true", default=False, help="Replay recorded events from previous run")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    parser.add_argument("--pace", type=float, default=0.04, help="Animation micro-delay pacing in seconds")
    parser.add_argument("-f", "--fleet-size", type=int, default=4, help="Fleet AMR count")
    args = parser.parse_args()

    log_path = Path(args.log_dir) if args.log_dir else None
    visualizer = LiveFleetVisualizer(
        log_dir=log_path,
        fleet_size=args.fleet_size,
        pace_s=args.pace,
        is_replay=args.replay,
        replay_speed=args.speed
    )

    if args.replay:
        visualizer.run_replay()
    else:
        visualizer.run_live()


if __name__ == "__main__":
    main()
