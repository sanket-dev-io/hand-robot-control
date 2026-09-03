"""
Created and Developed By : Sanket Kshirsagar
Role                      : Senior Research Engineer
Project                   : Hand Gesture Controlled Robotic System

Description:
Main launcher for the hand-tracking and robotic control system.
Provides real-time gesture-based robot operation, process
management, monitoring, recalibration, and safe shutdown.

Copyright © Sanket Kshirsagar
"""

import sys
import os
import signal
import subprocess
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HAND_TRACKER = os.path.join(SCRIPT_DIR, "hand_tracker.py")
ROBOT_CONTROLLER = os.path.join(SCRIPT_DIR, "robot_controller.py")


def main():
    for path in (HAND_TRACKER, ROBOT_CONTROLLER):
        if not os.path.isfile(path):
            print(f"[LAUNCHER] ERROR: missing file: {path}")
            sys.exit(1)

    print("[LAUNCHER] Starting hand tracker + robot controller…")
    print("[LAUNCHER] Gestures:")
    print("           • Closed fist (all fingers)  → recalibrate tracking origin")
    print("           • Thumb + middle finger      → STOP robot")
    print("           • Thumb + index finger       → gripper open/close")
    print("           • Hand window: q=quit, c=manual recalib")
    print("           • Robot window: ESC=quit, [ / ] = speed")
    print()

    # Start hand tracker first so the socket is listening
    hand_proc = subprocess.Popen(
        [sys.executable, HAND_TRACKER],
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    time.sleep(1.2)
    robot_proc = subprocess.Popen(
        [sys.executable, ROBOT_CONTROLLER, "--no-launch-tracker"],
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    def cleanup(signum=None, frame=None):
        print("\n[LAUNCHER] Shutting down…")
        for p in (robot_proc, hand_proc):
            if p.poll() is None:
                p.terminate()
        for p in (robot_proc, hand_proc):
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    while True:
        if hand_proc.poll() is not None:
            print("[LAUNCHER] Hand tracker exited")
            cleanup()
        if robot_proc.poll() is not None:
            print("[LAUNCHER] Robot controller exited")
            cleanup()
        time.sleep(0.4)


if __name__ == "__main__":
    main()
