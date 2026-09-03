"""
robot_controller.py
-------------------
Robot motion controller driven ONLY by hand tracking.

Starts hand_tracker.py as a subprocess and connects over TCP
(127.0.0.1:5005) to receive hand control values.

No joystick is used.

Gestures (from hand tracker):
  - Closed fist          → recalibrate (handled in tracker)
  - Thumb + index pinch  → gripper close/open
  - Thumb + middle pinch → STOP all robot motion
"""

import sys
import time
import socket
import struct
import subprocess
import os
import pygame
from MxTtoPycode import ZeroDelayMelfaController

# ── Network (must match hand_tracker.py) ─────────────────────────────
HOST = "127.0.0.1"
PORT = 5005
PACKET_FMT = "<ffffffi"   # x,y,z,rot,grip,stop,hand_present
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# ── Tunable parameters ───────────────────────────────────────────────
CONTROL_HZ          = 20
MAX_SPEED_MM_S      = 80.0
MAX_PTP_SPEED_PCT   = 40
MIN_PTP_SPEED_PCT   = 5
MAX_PTP_SPEED_CAP   = 80
SPEED_STEP_PCT      = 2
MOVE_THRESHOLD_MM   = 0.05

LIMITS = {
    0: (444.46, 1176.16),   # X
    1: (-674.81, 728.48),   # Y
    2: (474.09, 1115.13),   # Z
    5: (-154.69, 1.00),     # C
}

HOME_POSITION = [915.14, -380.0, 975.69, 179.09, -0.56, -92.52]

HAND_TRACKER_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "hand_tracker.py"
)


# ── UI ───────────────────────────────────────────────────────────────
WIN_W, WIN_H = 720, 580
BG     = (18, 18, 22)
FG     = (220, 220, 220)
GREEN  = (0, 210, 90)
RED    = (220, 50, 50)
YELLOW = (230, 190, 30)
ORANGE = (230, 130, 20)
CYAN   = (0, 200, 220)
AXIS_COLORS = [GREEN, (80, 160, 255), ORANGE, YELLOW]
BAR_W, BAR_H = 280, 22


def draw_bar(screen, font, x, y, label, value, color):
    screen.blit(font.render(label, True, FG), (x, y))
    bar_x, bar_y = x, y + 26
    pygame.draw.rect(screen, (50, 50, 55), (bar_x, bar_y, BAR_W, BAR_H))
    centre = bar_x + BAR_W // 2
    fill_w = int(abs(value) * (BAR_W // 2))
    if value >= 0:
        pygame.draw.rect(screen, color, (centre, bar_y, fill_w, BAR_H))
    else:
        pygame.draw.rect(screen, color, (centre - fill_w, bar_y, fill_w, BAR_H))
    pygame.draw.line(screen, (120, 120, 120), (centre, bar_y), (centre, bar_y + BAR_H), 1)
    screen.blit(font.render(f"{value:+.3f}", True, color), (bar_x + BAR_W + 8, bar_y))


def draw_pose_panel(screen, font, small_font, pose, limits, gripper_on,
                    current_max_speed, hand_present, hand_connected, stopped):
    labels = ["X", "Y", "Z", "A", "B", "C"]
    panel_x, panel_y = 30, 280
    screen.blit(font.render("CURRENT POSE", True, FG), (panel_x, panel_y))
    panel_y += 26
    for i, (lbl, val) in enumerate(zip(labels, pose)):
        if i in limits:
            lo, hi = limits[i]
            pct = (val - lo) / (hi - lo) if hi != lo else 0.5
            col = GREEN if 0.1 < pct < 0.9 else (RED if pct <= 0.1 or pct >= 0.9 else YELLOW)
            txt = f"{lbl}: {val:8.2f} [{lo:.1f} … {hi:.1f}]"
        else:
            col = FG
            txt = f"{lbl}: {val:8.2f}"
        screen.blit(small_font.render(txt, True, col), (panel_x, panel_y))
        panel_y += 22

    gcol = GREEN if gripper_on else RED
    screen.blit(font.render(
        "GRIPPER: CLOSED" if gripper_on else "GRIPPER: OPEN", True, gcol),
        (panel_x, panel_y + 4))

    panel_y += 32
    screen.blit(small_font.render(
        f"PTP Cap: {current_max_speed}%  (keys [ / ] to change)", True, YELLOW),
        (panel_x, panel_y))
    panel_y += 20
    hcol = CYAN if hand_present else (120, 120, 120)
    screen.blit(small_font.render(
        f"Hand: {'DETECTED' if hand_present else 'not present'}  "
        f"({'connected' if hand_connected else 'NO LINK'})",
        True, hcol), (panel_x, panel_y))
    panel_y += 20
    if stopped:
        screen.blit(small_font.render(
            "STATUS: STOPPED (thumb + middle finger)", True, RED),
            (panel_x, panel_y))
    else:
        screen.blit(small_font.render(
            "STATUS: RUNNING", True, GREEN),
            (panel_x, panel_y))


# ── Hand socket ──────────────────────────────────────────────────────
def connect_to_hand(timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            sock.connect((HOST, PORT))
            sock.setblocking(False)
            print(f"[ROBOT] Connected to hand tracker at {HOST}:{PORT}")
            return sock
        except (ConnectionRefusedError, OSError):
            time.sleep(0.3)
    print("[ROBOT] Could not connect to hand tracker (will retry in loop)")
    return None


def read_hand_packet(sock, buf):
    if sock is None:
        return None
    try:
        data = sock.recv(4096)
        if not data:
            return "disconnect"
        buf.extend(data)
    except BlockingIOError:
        pass
    except (ConnectionResetError, OSError):
        return "disconnect"

    if len(buf) >= PACKET_SIZE:
        packet = bytes(buf[:PACKET_SIZE])
        del buf[:PACKET_SIZE]
        while len(buf) >= PACKET_SIZE:
            packet = bytes(buf[:PACKET_SIZE])
            del buf[:PACKET_SIZE]
        return struct.unpack(PACKET_FMT, packet)
    return None


# ── Main ─────────────────────────────────────────────────────────────
def main(launch_tracker=True):
    hand_proc = None
    if launch_tracker and os.path.isfile(HAND_TRACKER_SCRIPT):
        print(f"[ROBOT] Starting hand tracker: {HAND_TRACKER_SCRIPT}")
        hand_proc = subprocess.Popen(
            [sys.executable, HAND_TRACKER_SCRIPT],
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
        time.sleep(1.5)
    elif launch_tracker:
        print(f"[ROBOT] WARNING: {HAND_TRACKER_SCRIPT} not found – "
              "run hand_tracker.py manually if needed")

    hand_sock = connect_to_hand()
    hand_buf = bytearray()

    hx = hy = hz = hrot = 0.0
    hgrip = False
    hstop = False
    hand_present = False
    hand_connected = hand_sock is not None

    print("[INIT] Connecting to robot...")
    bot = ZeroDelayMelfaController()
    if not bot.connect():
        if hand_proc:
            hand_proc.terminate()
        raise RuntimeError("Could not connect to robot!")

    print("[INIT] Moving to HOME (PTP)...")
    bot.move_instant(list(HOME_POSITION), speed_pct=MAX_PTP_SPEED_PCT, mode_type=1)
    bot.read_live_pose()
    current_pose = list(bot.current_pose) if bot.current_pose else list(HOME_POSITION)
    print("[INIT] Gripper open...")
    bot.set_gripper(False)
    gripper_on = False
    last_hand_grip = False

    current_max_speed = MAX_PTP_SPEED_PCT

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Hand Robot Controller")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 20, bold=True)
    sfont = pygame.font.SysFont("Arial", 16)

    dt = 1.0 / CONTROL_HZ
    running = True

    try:
        while running:
            clock.tick(CONTROL_HZ)
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_RIGHTBRACKET:
                        current_max_speed = min(MAX_PTP_SPEED_CAP,
                                                current_max_speed + SPEED_STEP_PCT)
                    elif event.key == pygame.K_LEFTBRACKET:
                        current_max_speed = max(MIN_PTP_SPEED_PCT,
                                                current_max_speed - SPEED_STEP_PCT)

            # ── read hand packet ─────────────────────────────────────
            if hand_sock is None:
                if int(time.time() * 2) % 4 == 0:
                    hand_sock = connect_to_hand(timeout=0.2)
                    hand_connected = hand_sock is not None
                    hand_buf.clear()
            else:
                result = read_hand_packet(hand_sock, hand_buf)
                if result == "disconnect":
                    print("[ROBOT] Hand tracker disconnected")
                    try:
                        hand_sock.close()
                    except Exception:
                        pass
                    hand_sock = None
                    hand_connected = False
                    hand_present = False
                    hx = hy = hz = hrot = 0.0
                    hgrip = False
                    hstop = False
                elif result is not None:
                    hx, hy, hz, hrot, grip_f, stop_f, present = result
                    hgrip = grip_f > 0.5
                    hstop = stop_f > 0.5
                    hand_present = present == 1
                    hand_connected = True

            # ── hand → robot deltas ──────────────────────────────────
            # Mapping: hand X → robot Y, hand Y → robot X,
            #          hand Z → robot Z, hand ROT → robot C
            if hand_present and not hstop:
                delta_y = hx * MAX_SPEED_MM_S * dt
                delta_x = hy * MAX_SPEED_MM_S * dt
                delta_z = hz * MAX_SPEED_MM_S * dt
                delta_c = hrot * MAX_SPEED_MM_S * dt
                magnitudes = [abs(hx), abs(hy), abs(hz), abs(hrot)]
                max_mag = max(magnitudes) if magnitudes else 0.0
                if max_mag > 0:
                    speed_pct = max(1, int(max_mag * current_max_speed))
                else:
                    speed_pct = 1
                    delta_x = delta_y = delta_z = delta_c = 0.0
            else:
                delta_x = delta_y = delta_z = delta_c = 0.0
                max_mag = 0.0
                speed_pct = 1

            new_pose = list(current_pose)
            new_pose[0] = max(LIMITS[0][0], min(LIMITS[0][1], new_pose[0] + delta_x))
            new_pose[1] = max(LIMITS[1][0], min(LIMITS[1][1], new_pose[1] + delta_y))
            new_pose[2] = max(LIMITS[2][0], min(LIMITS[2][1], new_pose[2] + delta_z))
            new_pose[5] = max(LIMITS[5][0], min(LIMITS[5][1], new_pose[5] + delta_c))

            moved = any(abs(new_pose[i] - current_pose[i]) > MOVE_THRESHOLD_MM
                        for i in range(6))
            if moved and hand_present and not hstop and max_mag > 0:
                bot.move_instant(new_pose, speed_pct=speed_pct, mode_type=1)
                current_pose = list(new_pose)

            # ── gripper from thumb+index pinch ───────────────────────
            if hand_present and not hstop:
                if hgrip and not last_hand_grip:
                    bot.set_gripper(True)
                    gripper_on = True
                elif not hgrip and last_hand_grip:
                    bot.set_gripper(False)
                    gripper_on = False
                last_hand_grip = hgrip
            else:
                last_hand_grip = False

            # ── UI ───────────────────────────────────────────────────
            screen.fill(BG)
            title = "Hand Robot Controller"
            if hstop:
                title += "  [STOPPED]"
            screen.blit(font.render(title, True, RED if hstop else FG), (20, 12))

            src = "HAND" if hand_present else "WAITING"
            if hstop:
                src = "STOPPED"
            screen.blit(sfont.render(f"Source: {src}", True,
                                     RED if hstop else (CYAN if hand_present else YELLOW)),
                        (WIN_W - 160, 14))
            screen.blit(sfont.render(
                f"Speed: {speed_pct}% | Cap: {current_max_speed}%", True, YELLOW),
                (WIN_W - 280, 36))

            labels = [
                f"handX → Y ({hx:+.3f})",
                f"handY → X ({hy:+.3f})",
                f"handZ → Z ({hz:+.3f})",
                f"handROT → C ({hrot:+.3f})",
            ]
            values = [hx, hy, hz, hrot]
            for i in range(4):
                col, row = i % 2, i // 2
                draw_bar(screen, sfont, 30 + col * 350, 55 + row * 80,
                         labels[i], values[i], AXIS_COLORS[i % 4])

            draw_pose_panel(screen, font, sfont, current_pose, LIMITS, gripper_on,
                            current_max_speed, hand_present, hand_connected, hstop)

            screen.blit(sfont.render(
                "ESC=quit | [ / ] = PTP speed | Fist=recalib | Thumb+Mid=STOP | Thumb+Idx=grip",
                True, (140, 140, 140)), (12, WIN_H - 26))
            pygame.display.flip()

            if hand_proc is not None and hand_proc.poll() is not None:
                if hand_connected:
                    print("[ROBOT] Hand tracker process exited")
                hand_connected = False
                hand_present = False

    finally:
        print("[SHUTDOWN] Cleaning up…")
        if hand_sock:
            try:
                hand_sock.close()
            except Exception:
                pass
        if hand_proc is not None and hand_proc.poll() is None:
            hand_proc.terminate()
            try:
                hand_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                hand_proc.kill()
        bot.set_gripper(False)
        try:
            bot.close()
        except Exception:
            pass
        pygame.quit()
        print("[SHUTDOWN] Done")


if __name__ == "__main__":
    # Allow launcher to pass --no-launch-tracker so tracker is not started twice
    launch = "--no-launch-tracker" not in sys.argv
    main(launch_tracker=launch)