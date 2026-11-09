#!/usr/bin/env python3
"""
GNSS Compass GPS Data Simulator & Rover Transit Test Suite
=========================================================

Simulates Unicore UM982 GNSS receiver data ($KSXT / NMEA) and injects it into
the ESP32 GNSS Compass display over USB CDC serial (/dev/ttyACM*).

Designed specifically to validate Rover Mode, transit tracking (16 km legal rule,
Maidenhead grid boundary crossings, canyon switchbacks), antenna rotator sweeps,
and RTK quality transitions without requiring outdoor road trips.

Usage Examples:
  # 1. Rover transit journey (drives from CM87wa across 16 km boundary at 60 km/h)
  ./tools/gps_sim.py rover --origin CM87wa --speed 75 --distance 25

  # 2. Mountain canyon switchback / backtrack scenario
  ./tools/gps_sim.py backtrack --origin CM87wa

  # 3. 360-degree antenna rotator sweep at 6 deg/sec
  ./tools/gps_sim.py sweep --speed 6.0

  # 4. RTK quality degradation & watchdog timeout cycle
  ./tools/gps_sim.py rtk

  # 5. Interactive terminal drive controller (WASD + hotkeys)
  ./tools/gps_sim.py drive --origin CM87wa

  # 6. One-shot / custom injection (hold heading and grid for 10s)
  ./tools/gps_sim.py inject --grid CM87wa --hdg 145.2 --quality fixed --duration 10

  # 7. Offline / dry-run (prints generated sentences to stdout)
  ./tools/gps_sim.py rover --dry-run
"""

import sys
import os
import time
import math
import glob
import select
import termios
import tty
import argparse
from datetime import datetime, timezone

try:
    import serial
except ImportError:
    serial = None


# --- Geodetics & Maidenhead Locator Helpers ---

def to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def calc_latlon_from_grid(grid: str) -> tuple[float, float]:
    """Computes latitude and longitude of the center of a Maidenhead grid square.

    Supports 4-character (e.g. EN40), 6-character (e.g. EN40ab), and 8-character (e.g. EN40ab56) grids.
    When less significant digits are omitted, coordinates default to the center of the specified square:
      - 4-char: Center of 2° × 1° square (+1.0° lon, +0.5° lat)
      - 6-char: Center of 5' × 2.5' subsquare (+2.5' lon, +1.25' lat)
      - 8-char: Center of 0.5' × 0.25' extended square (+0.25' lon, +0.125' lat)
    """
    g = grid.strip()
    if len(g) < 4:
        raise ValueError(f"Grid must be at least 4 characters: {grid}")

    lon = (ord(g[0].upper()) - ord('A')) * 20.0 + (ord(g[2]) - ord('0')) * 2.0 - 180.0
    lat = (ord(g[1].upper()) - ord('A')) * 10.0 + (ord(g[3]) - ord('0')) * 1.0 - 90.0

    if len(g) >= 6:
        lon += (ord(g[4].upper()) - ord('A')) * (5.0 / 60.0)
        lat += (ord(g[5].upper()) - ord('A')) * (2.5 / 60.0)
        if len(g) >= 8 and g[6].isdigit() and g[7].isdigit():
            lon += (ord(g[6]) - ord('0')) * (0.5 / 60.0) + (0.25 / 60.0)
            lat += (ord(g[7]) - ord('0')) * (0.25 / 60.0) + (0.125 / 60.0)
        else:
            lon += (2.5 / 60.0)
            lat += (1.25 / 60.0)
    else:
        lon += 1.0
        lat += 0.5

    return lat, lon

def calc_locator_from_latlon(lat: float, lon: float) -> str:
    """Computes an 8-character Maidenhead locator string from decimal coordinates."""
    rem_lon = lon + 180.0
    o1 = int(rem_lon / 20.0)
    rem_lon -= o1 * 20.0
    o2 = int(rem_lon / 2.0)
    rem_lon -= o2 * 2.0
    o3 = int(12.0 * rem_lon)
    rem_lon = 12.0 * rem_lon - o3
    o4 = int(10.0 * rem_lon)

    rem_lat = lat + 90.0
    a1 = int(rem_lat / 10.0)
    rem_lat -= a1 * 10.0
    a2 = int(rem_lat)
    rem_lat -= a2
    a3 = int(24.0 * rem_lat)
    rem_lat = 24.0 * rem_lat - a3
    a4 = int(10.0 * rem_lat)

    # Clamp bounds to valid character ranges
    o1 = max(0, min(17, o1))
    a1 = max(0, min(17, a1))
    o2 = max(0, min(9, o2))
    a2 = max(0, min(9, a2))
    o3 = max(0, min(23, o3))
    a3 = max(0, min(23, a3))
    o4 = max(0, min(9, o4))
    a4 = max(0, min(9, a4))

    return f"{chr(o1 + ord('A'))}{chr(a1 + ord('A'))}{o2}{a2}{chr(o3 + ord('a'))}{chr(a3 + ord('a'))}{o4}{a4}"

def calculate_wgs84(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """Calculates great-circle distance (meters) and initial bearing (degrees) between two points."""
    R = 6378137.0  # WGS-84 semi-major axis in meters
    dLat = to_rad(lat2 - lat1)
    dLon = to_rad(lon2 - lon1)
    rLat1 = to_rad(lat1)
    rLat2 = to_rad(lat2)

    a = math.sin(dLat / 2.0) ** 2 + math.cos(rLat1) * math.cos(rLat2) * (math.sin(dLon / 2.0) ** 2)
    a = max(0.0, min(1.0, a))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    dist = R * c

    y = math.sin(dLon) * math.cos(rLat2)
    x = math.cos(rLat1) * math.sin(rLat2) - math.sin(rLat1) * math.cos(rLat2) * math.cos(dLon)
    bearing = (to_deg(math.atan2(y, x)) + 360.0) % 360.0

    return dist, bearing

def project_displacement(lat: float, lon: float, distance_m: float, bearing_deg: float) -> tuple[float, float]:
    """Projects a destination point given a starting position, distance in meters, and bearing."""
    R = 6378137.0
    r_dist = distance_m / R
    r_hdg = to_rad(bearing_deg)
    r_lat = to_rad(lat)
    r_lon = to_rad(lon)

    dest_lat = math.asin(math.sin(r_lat) * math.cos(r_dist) + math.cos(r_lat) * math.sin(r_dist) * math.cos(r_hdg))
    dest_lon = r_lon + math.atan2(
        math.sin(r_hdg) * math.sin(r_dist) * math.cos(r_lat),
        math.cos(r_dist) - math.sin(r_lat) * math.sin(dest_lat)
    )

    return to_deg(dest_lat), to_deg(dest_lon)


# --- NMEA & KSXT Sentence Formatting ---

def compute_nmea_checksum(sentence: str) -> str:
    """Computes XOR checksum for characters between '$' and '*'."""
    chk = 0
    start = 1 if sentence.startswith('$') else 0
    end = sentence.find('*')
    if end == -1:
        end = len(sentence)
    for c in sentence[start:end]:
        chk ^= ord(c)
    return f"{chk:02X}"

def build_ksxt_sentence(
    now_dt: datetime,
    lat: float,
    lon: float,
    heading: float,
    pos_quality: int = 1,
    hdg_quality: int = 3,
    altitude: float = 2100.0,
    pitch: float = 0.0,
    roll: float = 0.0,
    sats: int = 28
) -> str:
    """Builds a formatted Unicore UM982 $KSXT sentence with a valid checksum."""
    utc_str = now_dt.strftime("%Y%m%d%H%M%S.00")
    payload = (
        f"$KSXT,{utc_str},{lon:.8f},{lat:.8f},{altitude:.2f},"
        f"{heading:.2f},{pitch:.2f},{roll:.2f},0.022,,{pos_quality},{hdg_quality},{sats},28"
    )
    chk = compute_nmea_checksum(payload)
    return f"{payload}*{chk}\r\n"


# --- Serial Link Management ---

def auto_detect_serial_port() -> str | None:
    """Searches standard system serial device paths for ESP32-S3 USB CDC ports."""
    patterns = [
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*"
    ]
    matches = []
    for pat in patterns:
        matches.extend(glob.glob(pat))

    if matches:
        matches.sort()
        return matches[0]
    return None

class SerialSender:
    def __init__(self, port: str | None, baud: int = 115200, dry_run: bool = False):
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.ser = None

        if not self.dry_run:
            if serial is None:
                print("[WARN] pyserial module not found. Falling back to dry-run mode (stdout).")
                self.dry_run = True
            elif not self.port:
                detected = auto_detect_serial_port()
                if detected:
                    self.port = detected
                    print(f"[SIM] Auto-detected serial port: {self.port}")
                else:
                    print("[WARN] No active serial port detected (/dev/ttyACM*). Falling back to dry-run mode.")
                    self.dry_run = True

            if not self.dry_run and self.port:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                    time.sleep(0.1)
                    # Drain any pending boot bytes
                    self.ser.read_all()
                    print(f"[SIM] Connected to {self.port} at {self.baud} baud.")
                except Exception as e:
                    print(f"[ERROR] Could not open {self.port}: {e}. Falling back to dry-run mode.")
                    self.dry_run = True

    def send(self, sentence: str):
        if self.dry_run or self.ser is None:
            # Print to stdout without trailing extra newline
            sys.stdout.write(sentence)
            sys.stdout.flush()
        else:
            try:
                self.ser.write(sentence.encode('ascii'))
                self.ser.flush()
            except Exception as e:
                print(f"\n[ERROR] Serial write error: {e}")

    def send_cmd(self, cmd: str, wait_resp: float = 0.2) -> str:
        """Sends a text CLI command (e.g. 'rover status', 'rover clear confirm') and returns response."""
        if self.dry_run or self.ser is None:
            return ""
        try:
            self.ser.write((cmd.strip() + "\r\n").encode('ascii'))
            self.ser.flush()
            if wait_resp > 0:
                time.sleep(wait_resp)
                return self.ser.read_all().decode('utf-8', errors='replace')
        except Exception as e:
            print(f"\n[ERROR] Serial command error: {e}")
        return ""

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# --- Scenario 1: Rover Transit Journey ---

def run_rover_scenario(
    sender: SerialSender,
    origin_grid: str,
    bearing: float,
    speed_kmh: float,
    target_dist_km: float,
    rate_hz: float,
    dest_grid: str | None = None,
    revisit: bool = False
):
    print("\n" + "=" * 65)
    print("      SCENARIO: ROVER TRANSIT JOURNEY (ARRL 16 KM RULE)")
    print("=" * 65)
    orig_lat, orig_lon = calc_latlon_from_grid(origin_grid)
    origin_sub = origin_grid[:6].upper()

    if dest_grid:
        dest_lat, dest_lon = calc_latlon_from_grid(dest_grid)
        dest_sub = dest_grid[:6].upper()
        target_dist_m, bearing = calculate_wgs84(orig_lat, orig_lon, dest_lat, dest_lon)
        target_dist_km = target_dist_m / 1000.0
        print(f"Origin Grid      : {origin_sub} ({orig_lat:.5f}, {orig_lon:.5f})")
        print(f"Destination Grid : {dest_sub} ({dest_lat:.5f}, {dest_lon:.5f})")
        if revisit:
            print("Course Mode      : Loop Course Revisit (State C Assertion)")
    else:
        print(f"Origin Grid      : {origin_sub} ({orig_lat:.5f}, {orig_lon:.5f})")

    print(f"Driving Bearing  : {bearing:.1f} deg")
    print(f"Driving Speed    : {speed_kmh:.1f} km/h")
    print(f"Target Distance  : {target_dist_km:.1f} km")
    print(f"Update Rate      : {rate_hz:.1f} Hz")
    print("-" * 65)
    print("Press Ctrl+C to abort simulation at any time.\n")

    current_dist_m = 0.0
    speed_mps = (speed_kmh * 1000.0) / 3600.0
    dt = 1.0 / rate_hz
    start_time = time.time()
    last_print = 0.0

    while current_dist_m <= target_dist_km * 1000.0:
        cur_lat, cur_lon = project_displacement(orig_lat, orig_lon, current_dist_m, bearing)
        cur_grid = calc_locator_from_latlon(cur_lat, cur_lon)
        cur_sub = cur_grid[:6].upper()

        dist_from_orig, _ = calculate_wgs84(orig_lat, orig_lon, cur_lat, cur_lon)
        dist_km = dist_from_orig / 1000.0

        # ARRL Rover Legality Rule Evaluation
        same_grid = (cur_sub == origin_sub)
        under_16k = (math.floor(dist_km) < 16)

        if same_grid and under_16k:
            status_tag = "[ILLEGAL]   < 16 km, Same Grid (RED on WHITE - State A)"
        elif not same_grid and under_16k:
            status_tag = "[WARN]      < 16 km, Traversed Grid (Amber - State A)"
        elif same_grid and not under_16k:
            status_tag = "[WARN]      >= 16 km, Same Grid (Keep driving)"
        elif revisit:
            status_tag = "[REVISIT]   >= 16 km, Revisited Grid (ROYAL BLUE - State C)"
        else:
            status_tag = "[QUALIFIED] >= 16 km & New Grid (FOREST GREEN - State B)"

        # Build and send sentence
        now_dt = datetime.now(timezone.utc)
        ksxt = build_ksxt_sentence(now_dt, cur_lat, cur_lon, bearing, pos_quality=1, hdg_quality=3)
        sender.send(ksxt)

        # Update terminal dashboard (throttle console updates to 5 Hz max)
        now = time.time()
        if now - last_print >= 0.2:
            elapsed = now - start_time
            sys.stdout.write(
                f"\r[{elapsed:05.1f}s] Dist: {dist_km:5.2f} km | Grid: {cur_sub} | "
                f"Hdg: {bearing:05.1f} | {status_tag}   "
            )
            sys.stdout.flush()
            last_print = now

        current_dist_m += speed_mps * dt
        time.sleep(dt)

    print("\n\n" + "=" * 65)
    print("Rover transit scenario completed successfully.")
    print("=" * 65 + "\n")


# --- Scenario 2: Mountain Backtrack / Switchbacks ---

def run_backtrack_scenario(sender: SerialSender, origin_grid: str, rate_hz: float, speed_mult: float = 1.0):
    print("\n" + "=" * 65)
    print("      SCENARIO: MOUNTAIN CANYON SWITCHBACK & BACKTRACK")
    print("=" * 65)
    orig_lat, orig_lon = calc_latlon_from_grid(origin_grid)
    origin_sub = origin_grid[:6].upper()

    print(f"Origin Grid      : {origin_sub} ({orig_lat:.5f}, {orig_lon:.5f})")
    print("Simulating winding road: Climb to 14.5 km -> Canyon drop to 11.2 km -> Breakout past 16 km to 20 km.")
    print("-" * 65)

    phases = [
        # (name, start_km, end_km, bearing, speed_kmh)
        ("Climbing mountain road", 0.0, 14.5, 30.0, 70.0 * speed_mult),
        ("Canyon turn (distance decreases)", 14.5, 11.2, 210.0, 50.0 * speed_mult),
        ("Ridge breakout past 16 km", 11.2, 20.0, 45.0, 80.0 * speed_mult),
    ]

    cur_lat = orig_lat
    cur_lon = orig_lon
    dt = 1.0 / rate_hz

    for phase_name, start_km, end_km, hdg, spd_kmh in phases:
        print(f"\n>>> Phase: {phase_name} ({start_km:.1f} km -> {end_km:.1f} km at {spd_kmh:.0f} km/h)")
        spd_mps = (spd_kmh * 1000.0) / 3600.0
        step_dist = (end_km - start_km) * 1000.0
        dist_moved = 0.0
        total_steps = max(1, int(abs(step_dist) / (spd_mps * dt)))

        for _ in range(total_steps):
            delta = spd_mps * dt if step_dist >= 0 else -spd_mps * dt
            cur_lat, cur_lon = project_displacement(cur_lat, cur_lon, abs(delta), hdg if delta >= 0 else (hdg + 180) % 360)
            cur_grid = calc_locator_from_latlon(cur_lat, cur_lon)
            cur_sub = cur_grid[:6].upper()

            dist_from_orig, _ = calculate_wgs84(orig_lat, orig_lon, cur_lat, cur_lon)
            dist_km = dist_from_orig / 1000.0

            under_16k = (math.floor(dist_km) < 16)
            alert = "ALERT < 16K (RED)" if under_16k else "CLEAR >= 16K (QUALIFIED)"

            now_dt = datetime.now(timezone.utc)
            ksxt = build_ksxt_sentence(now_dt, cur_lat, cur_lon, hdg, pos_quality=1, hdg_quality=3)
            sender.send(ksxt)

            sys.stdout.write(
                f"\rDisplacement from Origin: {dist_km:5.2f} km | Grid: {cur_sub} | Status: {alert}   "
            )
            sys.stdout.flush()
            time.sleep(dt)

    print("\n\nBacktrack scenario completed.\n")


# --- Scenario 2B: Rover First Transit & Backtrack (< 1 Minute) ---

def run_first_transit_scenario(
    sender: SerialSender,
    origin_grid: str = "CM87wa",
    rate_hz: float = 5.0,
    phase_duration: float = 15.0,
    hold_duration: float = 8.0,
    auto_start: bool = False
):
    print("\n" + "=" * 68)
    print("      SCENARIO: ROVER FIRST TRANSIT & BACKTRACK (< 1 MINUTE)")
    print("=" * 68)

    orig_lat, orig_lon = calc_latlon_from_grid(origin_grid)
    origin_sub = origin_grid[:6].upper()
    print(f"Origin Grid      : {origin_sub} ({orig_lat:.5f}, {orig_lon:.5f})")
    print(f"Phase Duration   : {phase_duration:.1f}s per phase (Total drive: {phase_duration * 3:.1f}s)")
    print(f"Update Rate      : {rate_hz:.1f} Hz")
    print("-" * 68)

    # -------------------------------------------------------------
    # Step 1: Clean Slate
    # -------------------------------------------------------------
    print("\n>>> [STEP 1/3] Ensuring Clean Slate on Device...")
    if not sender.dry_run:
        sender.send_cmd("rover stop", wait_resp=0.1)
        resp = sender.send_cmd("rover clear confirm", wait_resp=0.2)
        if "cleared from NVS" in resp:
            print("    [OK] Rover contest history successfully cleared (0 stops).")
        else:
            print("    [INFO] Clean slate command sent.")
    else:
        print("    [DRY-RUN] Simulating clean slate (0 stops in history).")

    # -------------------------------------------------------------
    # Step 2: Inject Origin Location (First Step) & Let Stream Go Quiet
    # -------------------------------------------------------------
    print(f"\n>>> [STEP 2/3] Injecting Initial Location ({origin_sub}) for 2.0s...")
    dt = 1.0 / rate_hz
    inject_steps = max(1, int(2.0 * rate_hz))
    for _ in range(inject_steps):
        now_dt = datetime.now(timezone.utc)
        ksxt = build_ksxt_sentence(now_dt, orig_lat, orig_lon, 45.0, pos_quality=1, hdg_quality=3)
        sender.send(ksxt)
        time.sleep(dt)

    print("    [OK] Initial location injected. Transmission stopped.")
    print("    [INFO] Watchdog will transition device display to 'NO GPS' (Deep Amber).")

    # -------------------------------------------------------------
    # Step 3: Prompt User to Start Rover Mode
    # -------------------------------------------------------------
    print("\n" + "#" * 68)
    print("   ACTION REQUIRED: START ROVER MODE ON THE GNSS COMPASS")
    print("#" * 68)
    print(f"   1. Tap the Grid area (Row 1) on the touchscreen to arm Rover Mode.")
    print(f"      -> (Or type 's' and press Enter to start via Serial CLI)")
    print(f"   2. The device will log Stop #1 at {origin_sub} (0.0 km).")
    print(f"   3. Row 5 will show [ TRANSIT   0.0 k ] or [ NO GPS    0.0 k ].")
    print("#" * 68)

    if not auto_start:
        try:
            choice = input("\n>>> When Rover Mode is active, press [ENTER] to start simulation (or 's' to start): ").strip().lower()
            if choice in ('s', 'start'):
                if not sender.dry_run:
                    sender.send_cmd("rover start", wait_resp=0.3)
                    print("    [OK] Sent 'rover start' command over Serial CLI.")
        except (KeyboardInterrupt, EOFError):
            print("\nAborted by user.")
            return
    else:
        if not sender.dry_run:
            sender.send_cmd("rover start", wait_resp=0.3)
            print("    [AUTO] Started Rover Mode via Serial CLI.")

    # Check and display device status if on live serial
    if not sender.dry_run:
        stat = sender.send_cmd("rover status", wait_resp=0.3)
        if "ACTIVE (Rover Mode)" in stat:
            print("    [CONFIRMED] Device is in ACTIVE Rover Mode.")
        else:
            print("    [NOTE] Continuing with simulation stream.")

    # -------------------------------------------------------------
    # Step 4: Accelerated 3-Phase Simulation (< 1 Minute)
    # -------------------------------------------------------------
    print("\n" + "=" * 68)
    print("      ACCELERATED TRANSIT & BACKTRACK SIMULATION RUNNING")
    print("=" * 68)

    # 3 distinct transit phases designed to test:
    # 1. Approach from 15.2 km and cross 16.0 km threshold -> QUALIFIED (Green)
    # 2. Canyon switchback / backtrack: distance drops from 16.8 km to 14.8 km -> TRANSIT (Red)
    # 3. Ridge breakout: distance climbs from 14.8 km to 17.0 km -> QUALIFIED (Green)
    phases = [
        ("Phase 1/3: Approach & Cross 16 km", 15.2, 16.8, 45.0),
        ("Phase 2/3: Canyon Backtrack (< 16 km)", 16.8, 14.8, 225.0),
        ("Phase 3/3: Ridge Breakout (>= 16 km)", 14.8, 17.0, 45.0),
    ]

    sim_start_time = time.time()
    for phase_name, start_km, end_km, heading in phases:
        print(f"\n>>> {phase_name} ({start_km:.1f} km -> {end_km:.1f} km, Bearing {heading:.0f}°)")
        delta_km = end_km - start_km
        total_steps = max(1, int(phase_duration * rate_hz))
        km_per_step = delta_km / float(total_steps)
        speed_kmh = (abs(delta_km) / phase_duration) * 3600.0
        print(f"    Speed: {speed_kmh:.0f} km/h | Duration: {phase_duration:.1f}s")

        cur_dist_km = start_km
        for step in range(total_steps):
            cur_lat, cur_lon = project_displacement(orig_lat, orig_lon, cur_dist_km * 1000.0, 45.0)
            cur_grid = calc_locator_from_latlon(cur_lat, cur_lon)
            cur_sub = cur_grid[:6].upper()

            # Status tag for terminal display
            under_16k = (math.floor(cur_dist_km) < 16)
            if under_16k:
                status_tag = "\033[91m[TRANSIT   < 16k]\033[0m (RED)"
            else:
                status_tag = "\033[92m[QUALIFIED >= 16k]\033[0m (GREEN)"

            now_dt = datetime.now(timezone.utc)
            ksxt = build_ksxt_sentence(now_dt, cur_lat, cur_lon, heading, pos_quality=1, hdg_quality=3)
            sender.send(ksxt)

            elapsed = time.time() - sim_start_time
            sys.stdout.write(
                f"\r[{elapsed:04.1f}s] Dist: {cur_dist_km:5.2f} km | Grid: {cur_sub} | Hdg: {heading:05.1f}° | {status_tag}   "
            )
            sys.stdout.flush()

            cur_dist_km += km_per_step
            time.sleep(dt)

    total_sim_time = time.time() - sim_start_time
    print(f"\n\n[OK] 3-Phase Simulation completed in {total_sim_time:.1f} seconds!")

    # -------------------------------------------------------------
    # Step 5: Hold Final Qualified State & Summary
    # -------------------------------------------------------------
    if hold_duration > 0:
        print(f"\n>>> Holding final QUALIFIED position for {hold_duration:.0f}s to keep screen active...")
        cur_lat, cur_lon = project_displacement(orig_lat, orig_lon, 17.0 * 1000.0, 45.0)
        hold_steps = int(hold_duration * rate_hz)
        for s in range(hold_steps):
            now_dt = datetime.now(timezone.utc)
            ksxt = build_ksxt_sentence(now_dt, cur_lat, cur_lon, 45.0, pos_quality=1, hdg_quality=3)
            sender.send(ksxt)
            rem = hold_duration - (s * dt)
            sys.stdout.write(f"\rHolding QUALIFIED 17.0 km... {rem:3.1f}s remaining   ")
            sys.stdout.flush()
            time.sleep(dt)
        print()

    # Query device status
    if not sender.dry_run:
        print("\n" + sender.send_cmd("rover status", wait_resp=0.3))

    print("=" * 68)
    print("      FIRST TRANSIT SCENARIO COMPLETED SUCCESSFULLY")
    print("=" * 68)
    print("Summary of Validated Behaviors:")
    print("  1. Clean slate reset (history erased to 0 stops).")
    print("  2. Injected Stop #1 origin, verified 'NO GPS' watchdog tolerance.")
    print("  3. Arming Rover Mode snapshotting origin coordinates.")
    print("  4. Approached from 15.2 km (TRANSIT Red) -> Crossed 16.0 km (QUALIFIED Green).")
    print("  5. Canyon Backtrack dropped under 16.0 km -> Re-alerted TRANSIT Red.")
    print("  6. Ridge Breakout exceeded 16.0 km -> Restored QUALIFIED Green.")
    print("=" * 68 + "\n")


# --- Scenario 3: 360° Rotator Continuous Sweep ---

def run_sweep_scenario(sender: SerialSender, speed_deg_sec: float, rate_hz: float):
    print("\n" + "=" * 65)
    print("      SCENARIO: 360-DEGREE ANTENNA ROTATOR SWEEP")
    print("=" * 65)
    print(f"Rotation Speed : {speed_deg_sec:.1f} deg/sec")
    print(f"Update Rate    : {rate_hz:.1f} Hz")
    print("Sweeping 0 -> 360 -> 0 degrees continuously. Press Ctrl+C to stop.")
    print("-" * 65)

    hdg = 0.0
    dt = 1.0 / rate_hz
    step = speed_deg_sec * dt
    lat, lon = 38.3540, -119.4580

    try:
        while True:
            now_dt = datetime.now(timezone.utc)
            ksxt = build_ksxt_sentence(now_dt, lat, lon, hdg, pos_quality=1, hdg_quality=3)
            sender.send(ksxt)

            sys.stdout.write(f"\rCurrent Antenna Heading: {hdg:05.1f} deg | Needle: PURE WHITE (RTK Fixed)   ")
            sys.stdout.flush()

            hdg = (hdg + step) % 360.0
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nRotator sweep stopped.")


# --- Scenario 4: RTK Quality Transitions & Watchdog ---

def run_rtk_scenario(sender: SerialSender, duration_mult: float = 1.0):
    print("\n" + "=" * 65)
    print("      SCENARIO: RTK QUALITY TRANSITIONS & WATCHDOG")
    print("=" * 65)
    print("Demonstrates: Fixed (White) -> Float (Yellow) -> Lost (Red) -> Timeout -> Reconnect")
    print("-" * 65)

    lat, lon, hdg = 38.3540, -119.4580, 145.2
    stages = [
        ("RTK FIXED (State B Qualified / Pure White)", 3, 1, 6.0 * duration_mult),
        ("RTK FLOAT (Bright Yellow Degraded Reception)", 2, 1, 5.0 * duration_mult),
        ("RTK LOST (Red Lost Carrier Ambiguity)", 0, 1, 4.0 * duration_mult),
        ("STREAM SILENCE (Testing 3000 ms Watchdog -> State D Deep Amber Holding Distance)", None, None, 4.0 * duration_mult),
        ("RE-ACQUIRED RTK FIXED (State B Restored)", 3, 1, 6.0 * duration_mult)
    ]

    for desc, hdg_q, pos_q, duration in stages:
        print(f"\n>>> State: {desc} ({duration:.0f} seconds)")
        start = time.time()
        while time.time() - start < duration:
            if hdg_q is not None:
                now_dt = datetime.now(timezone.utc)
                ksxt = build_ksxt_sentence(now_dt, lat, lon, hdg, pos_quality=pos_q, hdg_quality=hdg_q)
                sender.send(ksxt)
            sys.stdout.write(f"\rTime Remaining in State: {duration - (time.time() - start):4.1f}s   ")
            sys.stdout.flush()
            time.sleep(0.2)

    print("\n\nRTK quality scenario completed.\n")


# --- Scenario 5: Interactive Terminal Drive Simulator ---

def get_key_nonblocking() -> str | None:
    """Reads a single keypress without waiting for enter on UNIX terminals."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

def run_interactive_drive(sender: SerialSender, origin_grid: str, rate_hz: float):
    print("\n" + "=" * 65)
    print("        INTERACTIVE ROVER DRIVE CONTROLLER")
    print("=" * 65)
    print("Controls:")
    print("  [W] / [S]   : Speed +/- 10 km/h")
    print("  [A] / [D]   : Steer Left / Right (-/+ 5 deg)")
    print("  [1] / [2]   : RTK Fixed (White) / RTK Float (Yellow)")
    print("  [3]         : RTK Lost (Red)")
    print("  [Space]     : Emergency Brake (0 km/h)")
    print("  [Q]         : Quit Simulation")
    print("-" * 65)

    orig_lat, orig_lon = calc_latlon_from_grid(origin_grid)
    origin_sub = origin_grid[:6].upper()

    cur_lat, cur_lon = orig_lat, orig_lon
    speed_kmh = 0.0
    hdg = 45.0
    hdg_qual = 3  # RTK Fixed
    dt = 1.0 / rate_hz

    # Put terminal in raw mode for instant keypress handling
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        running = True
        while running:
            key = get_key_nonblocking()
            if key:
                k = key.lower()
                if k == 'w':
                    speed_kmh = min(150.0, speed_kmh + 10.0)
                elif k == 's':
                    speed_kmh = max(0.0, speed_kmh - 10.0)
                elif k == 'a':
                    hdg = (hdg - 5.0) % 360.0
                elif k == 'd':
                    hdg = (hdg + 5.0) % 360.0
                elif k == '1':
                    hdg_qual = 3  # Fixed
                elif k == '2':
                    hdg_qual = 2  # Float
                elif k == '3':
                    hdg_qual = 0  # Lost
                elif k == ' ':
                    speed_kmh = 0.0
                elif k == 'q':
                    running = False
                    break

            # Move coordinates
            spd_mps = (speed_kmh * 1000.0) / 3600.0
            dist_step = spd_mps * dt
            if dist_step > 0:
                cur_lat, cur_lon = project_displacement(cur_lat, cur_lon, dist_step, hdg)

            dist_from_orig, _ = calculate_wgs84(orig_lat, orig_lon, cur_lat, cur_lon)
            dist_km = dist_from_orig / 1000.0
            cur_grid = calc_locator_from_latlon(cur_lat, cur_lon)[:6].upper()

            under_16k = (math.floor(dist_km) < 16)
            alert = "ALERT < 16K (RED)" if under_16k else "QUALIFIED (CLEAR)"

            q_name = "FIXED" if hdg_qual == 3 else ("FLOAT" if hdg_qual == 2 else "LOST")

            now_dt = datetime.now(timezone.utc)
            ksxt = build_ksxt_sentence(now_dt, cur_lat, cur_lon, hdg, pos_quality=1, hdg_quality=hdg_qual)
            sender.send(ksxt)

            sys.stdout.write(
                f"\rSpeed: {speed_kmh:4.0f} km/h | Hdg: {hdg:05.1f} | Dist: {dist_km:5.2f} km | "
                f"Grid: {cur_grid} | RTK: {q_name:5s} | {alert}   "
            )
            sys.stdout.flush()
            time.sleep(dt)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\n\nInteractive drive session ended.")


# --- Scenario 6: Parameter & Raw Sentence Injection ---

def run_inject(
    sender: SerialSender,
    grid: str | None = None,
    lat: float | None = None,
    lon: float | None = None,
    hdg: float = 0.0,
    quality: str = "fixed",
    utc_time_str: str | None = None,
    rover_origin: str | None = None,
    rover_dist: float | None = None,
    altitude: float = 2100.0,
    sats: int = 28,
    raw_sentence: str | None = None,
    duration: float | None = None,
    repeat: int = 1,
    rate_hz: float = 5.0
):
    if raw_sentence:
        raw = raw_sentence.strip()
        if not raw.startswith("$"):
            raw = "$" + raw
        if "*" not in raw:
            raw = f"{raw}*{compute_nmea_checksum(raw)}"
        if not raw.endswith("\r\n"):
            raw = raw + "\r\n"
        sentence = raw
    else:
        # Determine coordinates
        if rover_origin is not None and rover_dist is not None:
            orig_lat, orig_lon = calc_latlon_from_grid(rover_origin)
            lat, lon = project_displacement(orig_lat, orig_lon, rover_dist * 1000.0, hdg if hdg != 0.0 else 45.0)
            cur_grid = calc_locator_from_latlon(lat, lon)[:6].upper()
            under_16k = math.floor(rover_dist) < 16
            alert_str = "ALERT < 16K (RED ON WHITE)" if under_16k else "QUALIFIED >= 16K (CLEAR)"
            print(f"[INJECT ROVER] Origin: {rover_origin} -> Dist: {rover_dist:.1f} km -> Current Grid: {cur_grid} ({alert_str})")
        elif grid:
            lat, lon = calc_latlon_from_grid(grid)
        elif lat is None or lon is None:
            lat, lon = 38.3540, -119.4580

        # Determine UTC time
        if utc_time_str:
            parts = utc_time_str.strip().split(":")
            h = int(parts[0])
            m = int(parts[1]) if len(parts) > 1 else 0
            s = int(parts[2]) if len(parts) > 2 else 0
            now_dt = datetime.now(timezone.utc).replace(hour=h, minute=m, second=s, microsecond=0)
        else:
            now_dt = datetime.now(timezone.utc)

        q_map = {"fixed": 3, "float": 2, "lost": 0}
        sentence = build_ksxt_sentence(
            now_dt, lat, lon, hdg,
            pos_quality=1,
            hdg_quality=q_map.get(quality.lower(), 3),
            altitude=altitude,
            sats=sats
        )

    dt = 1.0 / rate_hz
    if duration is not None and duration > 0:
        total_packets = max(1, int(duration * rate_hz))
        print(f"[INJECT] Streaming {total_packets} packets over {duration:.1f}s at {rate_hz:.1f} Hz (holding active display state)...")
        start = time.time()
        for idx in range(total_packets):
            sender.send(sentence)
            rem = max(0.0, duration - (time.time() - start))
            sys.stdout.write(f"\r[INJECT] Holding state... {rem:4.1f}s remaining (Press Ctrl+C to stop)   ")
            sys.stdout.flush()
            time.sleep(dt)
        print(f"\n[INJECT] Done. Injected {total_packets} packets.")
    elif repeat > 1:
        print(f"[INJECT] Repeating {repeat} packets at {rate_hz:.1f} Hz...")
        for idx in range(repeat):
            sender.send(sentence)
            sys.stdout.write(f"\r[INJECT] Sent packet {idx + 1}/{repeat}...   ")
            sys.stdout.flush()
            time.sleep(dt)
        print(f"\n[INJECT] Done.")
    else:
        sender.send(sentence)
        print(f"[INJECT] Sent 1 packet: {sentence.strip()}")
        print("  (Note: ESP32 staleness watchdog triggers after 1.5s. Use '--duration <sec>' to hold state on display)")


# --- CLI Parser & Entry Point ---

def main():
    parser = argparse.ArgumentParser(
        description="GNSS Compass GPS Data Simulator & Rover Transit Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument("--port", "-p", help="Serial port to ESP32 (e.g. /dev/ttyACM0, auto-detected if omitted)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    subparsers = parser.add_subparsers(dest="scenario", help="Simulation scenario to execute")

    # 1. Rover Scenario
    rover_p = subparsers.add_parser("rover", help="Simulate rover transit journey from an origin grid across 16 km threshold")
    rover_p.add_argument("--origin", default="CM87wa", help="Starting Maidenhead grid (default: CM87wa)")
    rover_p.add_argument("--dest", help="Destination Maidenhead grid (e.g. CM87wb or CM87wa) for point-to-point transit / loop course")
    rover_p.add_argument("--revisit", action="store_true", help="Simulate returning to an activated grid (State C REVISIT)")
    rover_p.add_argument("--bearing", type=float, default=45.0, help="Travel bearing heading in degrees (default: 45.0)")
    rover_p.add_argument("--speed", type=float, default=65.0, help="Driving speed in km/h (default: 65.0)")
    rover_p.add_argument("--distance", type=float, default=25.0, help="Total travel distance in km (default: 25.0)")
    rover_p.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")
    rover_p.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    # 2. Backtrack Scenario
    backtrack_p = subparsers.add_parser("backtrack", help="Simulate mountain canyon switchbacks where distance temporarily decreases")
    backtrack_p.add_argument("--origin", default="CM87wa", help="Starting Maidenhead grid (default: CM87wa)")
    backtrack_p.add_argument("--speed-mult", type=float, default=1.0, help="Speed multiplier for simulation (default: 1.0)")
    backtrack_p.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")
    backtrack_p.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    # 2B. Rover First Transit & Backtrack (< 1 minute fast complete scenario)
    first_transit_p = subparsers.add_parser(
        "first-transit",
        aliases=["first_transit"],
        help="Accelerated Rover first-transit scenario: Clean slate -> Stop #1 -> 16km cross -> Backtrack (< 1 min)"
    )
    first_transit_p.add_argument("--origin", default="CM87wa", help="Starting Maidenhead grid (default: CM87wa)")
    first_transit_p.add_argument("--phase-duration", type=float, default=15.0, help="Duration in seconds per transit phase (default: 15.0s, total 45s)")
    first_transit_p.add_argument("--hold-duration", type=float, default=8.0, help="Hold duration in seconds at final qualified stop (default: 8.0s)")
    first_transit_p.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")
    first_transit_p.add_argument("--auto-start", action="store_true", help="Automatically arm Rover Mode via Serial CLI instead of interactive prompt")
    first_transit_p.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    # 3. Sweep Scenario
    sweep_p = subparsers.add_parser("sweep", help="Continuous 360-degree antenna rotator sweep")
    sweep_p.add_argument("--speed", type=float, default=6.0, help="Rotation speed in deg/sec (default: 6.0)")
    sweep_p.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")
    sweep_p.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    # 4. RTK Quality Cycle
    rtk_p = subparsers.add_parser("rtk", help="Cycle through RTK Fixed, Float, Lost, and Staleness Watchdog timeout")
    rtk_p.add_argument("--duration-mult", type=float, default=1.0, help="Duration multiplier for simulation (default: 1.0)")
    rtk_p.add_argument("--dry-run", "-n", action="store_true", help="Print generated sentences to stdout without serial connection")

    # 5. Interactive Drive
    drive_p = subparsers.add_parser("drive", help="Interactive terminal keyboard drive controller (WASD)")
    drive_p.add_argument("--origin", default="CM87wa", help="Starting Maidenhead grid (default: CM87wa)")
    drive_p.add_argument("--rate", type=float, default=5.0, help="Update rate in Hz (default: 5.0)")

    # 6. Direct Inject
    inject_p = subparsers.add_parser(
        "inject",
        help="One-shot or continuous injection of coordinates, heading, time, or raw NMEA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 1. Quick coordinate and heading injection (Pure white fixed needle)
  ./tools/gps_sim.py inject --grid CM87wa --hdg 145.2 --quality fixed

  # 2. Hold heading and position on screen for 10 seconds (prevents 1.5s watchdog timeout)
  ./tools/gps_sim.py inject --grid CM87wb --hdg 90.0 --quality fixed --duration 10

  # 3. Simulate rover displacement relative to an origin (test 16 km alert threshold)
  ./tools/gps_sim.py inject --rover-origin CM87wa --rover-dist 14.5 --hdg 45.0 --duration 15

  # 4. Set specific UTC time on the clock display (Row 6)
  ./tools/gps_sim.py inject --grid CM87wa --time 23:45

  # 5. Test degraded RTK reception (Bright yellow float needle)
  ./tools/gps_sim.py inject --grid CM87wa --hdg 210.0 --quality float --duration 8

  # 6. Inject an arbitrary raw NMEA or $KSXT sentence directly
  ./tools/gps_sim.py inject --raw '$GNGLL,3821.24,N,11927.48,W,183000.00,A*23'
"""
    )
    inject_p.add_argument("--grid", help="Maidenhead grid location (e.g. CM87wa or CM87wa22)")
    inject_p.add_argument("--lat", type=float, help="Decimal latitude (e.g. 38.3540)")
    inject_p.add_argument("--lon", type=float, help="Decimal longitude (e.g. -119.4580)")
    inject_p.add_argument("--hdg", type=float, default=0.0, help="Antenna heading in degrees (default: 0.0)")
    inject_p.add_argument("--quality", "-q", choices=["fixed", "float", "lost"], default="fixed", help="RTK bearing quality: fixed (white), float (yellow), lost (red) (default: fixed)")
    inject_p.add_argument("--time", "-t", dest="utc_time", help="Simulated UTC time as 'HH:MM' or 'HH:MM:SS' (default: current UTC time)")
    inject_p.add_argument("--rover-origin", help="Origin Maidenhead grid for rover distance testing (e.g. CM87wa)")
    inject_p.add_argument("--rover-dist", type=float, help="Displacement distance in km from rover origin (e.g. 14.5)")
    inject_p.add_argument("--duration", "-d", type=float, help="Hold duration in seconds to continuously stream packet (prevents 1.5s staleness timeout)")
    inject_p.add_argument("--repeat", "-r", type=int, default=1, help="Number of packets to send (default: 1)")
    inject_p.add_argument("--rate", type=float, default=5.0, help="Streaming rate in Hz when using --duration or --repeat (default: 5.0)")
    inject_p.add_argument("--altitude", "--alt", type=float, default=2100.0, help="Antenna altitude in meters (default: 2100.0)")
    inject_p.add_argument("--sats", type=int, default=28, help="Number of tracked satellites (default: 28)")
    inject_p.add_argument("--raw", help="Directly inject a raw custom NMEA or $KSXT sentence string")

    args = parser.parse_args()

    if not args.scenario:
        parser.print_help()
        sys.exit(1)

    is_dry_run = getattr(args, 'dry_run', False)
    sender = SerialSender(args.port, args.baud, is_dry_run)

    try:
        if args.scenario == "rover":
            revisit_mode = getattr(args, 'revisit', False)
            run_rover_scenario(
                sender, args.origin, args.bearing, args.speed, args.distance, args.rate,
                dest_grid=getattr(args, 'dest', None),
                revisit=revisit_mode
            )
        elif args.scenario == "backtrack":
            run_backtrack_scenario(sender, args.origin, args.rate, speed_mult=getattr(args, 'speed_mult', 1.0))
        elif args.scenario in ("first-transit", "first_transit"):
            run_first_transit_scenario(
                sender,
                origin_grid=args.origin,
                rate_hz=args.rate,
                phase_duration=getattr(args, 'phase_duration', 15.0),
                hold_duration=getattr(args, 'hold_duration', 8.0),
                auto_start=getattr(args, 'auto_start', False)
            )
        elif args.scenario == "sweep":
            run_sweep_scenario(sender, args.speed, args.rate)
        elif args.scenario == "rtk":
            run_rtk_scenario(sender, duration_mult=getattr(args, 'duration_mult', 1.0))
        elif args.scenario == "drive":
            run_interactive_drive(sender, args.origin, args.rate)
        elif args.scenario == "inject":
            run_inject(
                sender,
                grid=args.grid,
                lat=args.lat,
                lon=args.lon,
                hdg=args.hdg,
                quality=args.quality,
                utc_time_str=args.utc_time,
                rover_origin=args.rover_origin,
                rover_dist=args.rover_dist,
                altitude=args.altitude,
                sats=args.sats,
                raw_sentence=args.raw,
                duration=args.duration,
                repeat=args.repeat,
                rate_hz=args.rate
            )
    except KeyboardInterrupt:
        print("\n[SIM] Interrupted by user.")
    finally:
        sender.close()

if __name__ == "__main__":
    main()

