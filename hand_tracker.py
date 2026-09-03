"""
hand_tracker.py
---------------
Standalone hand-tracking process (MediaPipe + OpenCV).

Publishes control values over a local TCP socket so the robot
controller can read them in real time.

Packet: x, y, z, rot, grip, stop, hand_present  (6 floats + 1 int)

Gestures:
  - Closed fist (all fingers curled)  → recalibrate centre / Z / rotation
  - Thumb + index pinch               → gripper close
  - Thumb + middle finger pinch       → STOP robot motion
  - Open hand / release               → resume motion, gripper open

Keys in the OpenCV window:
  q  – quit
  c  – reset centre / Z / rotation reference
"""

import socket
import struct
import time
import math
import cv2
import mediapipe as mp

# ── Network ──────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 5005

# Packet: 6 floats (x, y, z, rot, grip, stop) + 1 int (hand_present)
PACKET_FMT = "<ffffffi"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# ── MediaPipe / camera settings ──────────────────────────────────────
XY_SCALE = 5.0
Z_SCALE = 10.0
ROTATION_SCALE = 90.0
PINCH_ON = 0.055
PINCH_OFF = 0.075
MIDDLE_PINCH_ON = 0.055
MIDDLE_PINCH_OFF = 0.080
FIST_TIP_TO_MCP_RATIO = 0.55   # tip closer than this fraction of open length → curled
FIST_HOLD_FRAMES = 8           # consecutive frames of fist before recalibrate


def finger_curled(landmarks, tip_idx, pip_idx, mcp_idx):
    """True if finger tip is folded toward the MCP (curled)."""
    tip = landmarks[tip_idx]
    pip = landmarks[pip_idx]
    mcp = landmarks[mcp_idx]
    # distance tip→mcp vs pip→mcp; when curled tip is near mcp
    tip_mcp = math.sqrt((tip.x - mcp.x) ** 2 + (tip.y - mcp.y) ** 2)
    pip_mcp = math.sqrt((pip.x - mcp.x) ** 2 + (pip.y - mcp.y) ** 2)
    if pip_mcp < 1e-6:
        return False
    return (tip_mcp / pip_mcp) < FIST_TIP_TO_MCP_RATIO


def is_fist(landmarks):
    """All four fingers (index–pinky) curled. Thumb ignored for fist."""
    # index 8,6,5  middle 12,10,9  ring 16,14,13  pinky 20,18,17
    return (
        finger_curled(landmarks, 8, 6, 5)
        and finger_curled(landmarks, 12, 10, 9)
        and finger_curled(landmarks, 16, 14, 13)
        and finger_curled(landmarks, 20, 18, 17)
    )


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1.0)
    print(f"[HAND] Listening on {HOST}:{PORT} – waiting for robot controller…")

    conn = None

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7,
    )

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[HAND] ERROR: Camera could not be opened")
        return

    print("[HAND] Camera opened")
    print("[HAND] Gestures: closed FIST = recalibrate | thumb+middle = STOP | thumb+index = grip")

    center_x = center_y = center_hand_size = center_angle = None
    grip_on = False
    stop_on = False
    fist_frames = 0
    last_recalib_time = 0.0

    x_value = y_value = z_value = rotation_value = 0.0
    hand_present = 0

    running = True
    while running:
        if conn is None:
            try:
                conn, addr = server.accept()
                conn.setblocking(False)
                print(f"[HAND] Robot controller connected from {addr}")
            except socket.timeout:
                pass

        success, frame = cap.read()
        if not success:
            continue

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)
        h, w, _ = frame.shape

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
            lm = hand.landmark

            wrist = lm[0]
            thumb_tip = lm[4]
            index_tip = lm[8]
            middle_tip = lm[12]
            middle_mcp = lm[9]

            # ── Fist → recalibrate ───────────────────────────────────
            fist = is_fist(lm)
            if fist:
                fist_frames += 1
            else:
                fist_frames = 0

            now = time.time()
            if fist_frames >= FIST_HOLD_FRAMES and (now - last_recalib_time) > 1.0:
                center_x = center_y = center_hand_size = center_angle = None
                x_value = y_value = z_value = rotation_value = 0.0
                fist_frames = 0
                last_recalib_time = now
                print("[HAND] FIST detected → CENTER / Z / ROTATION RESET")

            # X / Y
            curr_x = middle_mcp.x
            curr_y = middle_mcp.y
            if center_x is None:
                center_x = curr_x
                center_y = curr_y
            x_value = max(-1.0, min(1.0, (curr_x - center_x) * XY_SCALE))
            y_value = max(-1.0, min(1.0, (curr_y - center_y) * XY_SCALE))

            # Z
            hand_size = math.sqrt(
                (wrist.x - middle_mcp.x) ** 2 + (wrist.y - middle_mcp.y) ** 2
            )
            if center_hand_size is None:
                center_hand_size = hand_size
            z_value = max(-1.0, min(1.0, (hand_size - center_hand_size) * Z_SCALE))

            # Rotation
            rot_dx = middle_mcp.x - wrist.x
            rot_dy = middle_mcp.y - wrist.y
            current_angle = math.degrees(math.atan2(rot_dy, rot_dx))
            if center_angle is None:
                center_angle = current_angle
            angle_diff = current_angle - center_angle
            if angle_diff > 180:
                angle_diff -= 360
            if angle_diff < -180:
                angle_diff += 360
            rotation_value = max(-1.0, min(1.0, angle_diff / ROTATION_SCALE))

            # Thumb + index → grip
            pinch_index = math.sqrt(
                (thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2
            )
            if not grip_on and pinch_index < PINCH_ON:
                grip_on = True
            elif grip_on and pinch_index > PINCH_OFF:
                grip_on = False

            # Thumb + middle → STOP
            pinch_middle = math.sqrt(
                (thumb_tip.x - middle_tip.x) ** 2 + (thumb_tip.y - middle_tip.y) ** 2
            )
            if not stop_on and pinch_middle < MIDDLE_PINCH_ON:
                stop_on = True
            elif stop_on and pinch_middle > MIDDLE_PINCH_OFF:
                stop_on = False

            # While fist is held, zero motion so recalib doesn't jump
            if fist:
                x_value = y_value = z_value = rotation_value = 0.0

            hand_present = 1

            # Visuals
            px, py = int(curr_x * w), int(curr_y * h)
            cx, cy = int(center_x * w), int(center_y * h)
            cv2.circle(frame, (px, py), 10, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 12, (255, 0, 0), 2)
            cv2.line(frame, (cx, cy), (px, py), (255, 255, 0), 2)

            cv2.putText(frame, f"X = {x_value:+.2f}", (20, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.putText(frame, f"Y = {y_value:+.2f}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.putText(frame, f"Z = {z_value:+.2f}", (20, 115),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.putText(frame, f"ROT = {rotation_value:+.2f}", (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
            cv2.putText(frame, f"ANGLE = {angle_diff:+.1f} deg", (20, 185),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            grip_text = "GRIP = ON" if grip_on else "GRIP = OFF"
            cv2.putText(frame, grip_text, (20, 225),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if grip_on else (0, 0, 255), 3)

            stop_text = "STOP = ON  (thumb+middle)" if stop_on else "STOP = OFF"
            cv2.putText(frame, stop_text, (20, 265),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 0, 255) if stop_on else (180, 180, 180), 2)

            if fist:
                cv2.putText(frame, "FIST → RECALIBRATING", (20, 305),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            cv2.putText(frame, f"PINCH idx={pinch_index:.3f} mid={pinch_middle:.3f}",
                        (20, 345), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            x_value = y_value = z_value = rotation_value = 0.0
            grip_on = False
            stop_on = False
            fist_frames = 0
            hand_present = 0
            cv2.putText(frame, "HAND NOT DETECTED", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        if conn is not None:
            packet = struct.pack(
                PACKET_FMT,
                x_value, -z_value, -y_value, rotation_value,
                1.0 if grip_on else 0.0,
                1.0 if stop_on else 0.0,
                hand_present,
            )
            try:
                conn.sendall(packet)
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("[HAND] Robot controller disconnected")
                conn.close()
                conn = None

        cv2.imshow("3D Hand Robot Controller", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            running = False
        elif key == ord("c"):
            center_x = center_y = center_hand_size = center_angle = None
            x_value = y_value = z_value = rotation_value = 0.0
            print("[HAND] CENTER / Z / ROTATION RESET (key c)")

    if conn is not None:
        conn.close()
    server.close()
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    print("[HAND] Exited")


if __name__ == "__main__":
    main()