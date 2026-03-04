# Script lama kelompok G
# Author: Alif Gibran Muhammad Ervin (5026251070)
#
# Logika: Camera detect tag0(payload) -> send posisi tag0 ke MCU sampai tag0 dekat (0,0) ->
# send "PAYLOAD" ke MCU -> tunggu MCU request drop -> track tag1(droppoint) sampai MCU send "DONE"


import math
import cv2
import numpy as np
from pupil_apriltags import Detector
import serial
import time

# ================= CAMERA CONFIG =================
TAG_SIZE = 0.108  # meters
DIST_THRESHOLD_CM = 1

calib = np.load("calib.npz")
cameraMatrix = calib["cameraMatrix"]

fx = cameraMatrix[0, 0]
fy = cameraMatrix[1, 1]
cx = cameraMatrix[0, 2]
cy = cameraMatrix[1, 2]

camera_params = (fx, fy, cx, cy)

# ================= SERIAL =================
SEND_INTERVAL = 1  # seconds
last_target_sent_time = 0.0

ser = serial.Serial("COM5", 115200, timeout=0.1)
time.sleep(2)


# ================= CAMERA =================
cap = cv2.VideoCapture(0)

at_detector = Detector(  # at = apriltag
    families="tagStandard41h12",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

# ================= STATE MACHINE abal abal =================
STATE_WAIT_PICK = 0
STATE_WAIT_PICK_DONE = 1
STATE_TRACK_DROP = 2
STATE_WAIT_FINAL_DONE = 3

state = STATE_WAIT_PICK
last_msg = ""
payload_signal_sent = False


def draw_tag_visual(frame, det):
    pts = np.array(det.corners, dtype=np.int32)

    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    cx_pix = int(det.center[0])
    cy_pix = int(det.center[1])

    cv2.circle(frame, (cx_pix, cy_pix), 5, (0, 255, 0), -1)

    t = det.pose_t
    x = float(t[0])
    y = float(t[1])
    z = float(t[2])

    line0 = f"ID: {det.tag_id}"
    line1 = f"x: {x:.3f} m"
    line2 = f"y: {y:.3f} m"
    line3 = f"z: {z:.3f} m"

    cv2.putText(
        frame,
        line0,
        (cx_pix + 10, cy_pix - 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
    )

    cv2.putText(
        frame,
        line1,
        (cx_pix + 10, cy_pix - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
    )

    cv2.putText(
        frame,
        line2,
        (cx_pix + 10, cy_pix + 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
    )

    cv2.putText(
        frame,
        line3,
        (cx_pix + 10, cy_pix + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
    )


def meter_to_cm(offset_m: float) -> int:
    return int(offset_m * 100.0)


def find_tag(detections, tag_id):
    for d in detections:
        if d.tag_id == tag_id:
            return d
    return None


while True:
    ret, frame = cap.read()
    if not ret:
        break

    msg = ""

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = at_detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=TAG_SIZE,
    )

    current_time = time.time()

    for det in detections:
        draw_tag_visual(frame, det)

    # ================= SERIAL READ =================
    if ser.in_waiting:
        msg = ser.readline().decode(errors="ignore").strip()
        print("MCU:", msg)

    # ================= WAIT PICK =================
    if state == STATE_WAIT_PICK:
        det0 = find_tag(detections, 0)

        if det0 is not None:
            t = det0.pose_t
            x_cm = meter_to_cm(float(t[0]))
            y_cm = meter_to_cm(float(t[1]))

            distance = math.sqrt(x_cm**2 + y_cm**2)

            if distance <= DIST_THRESHOLD_CM:
                if not payload_signal_sent:
                    ser.write(b"PAYLOAD\n")
                    print("ID0 near (0,0) -> PAYLOAD sent")
                    payload_signal_sent = True
                    state = STATE_WAIT_PICK_DONE

            if current_time - last_target_sent_time >= SEND_INTERVAL:
                command = f"TARGET {x_cm} {y_cm}\n"
                ser.write(command.encode())
                print("Sent PICK:", command.strip())
                last_target_sent_time = current_time

    # ================= WAIT PICK DONE =================
    if state == STATE_WAIT_PICK_DONE and msg == "REQUEST_DROP":
        print("MCU requested DROP")
        state = STATE_TRACK_DROP

    # ================= TRACK DROP =================

    if state == STATE_TRACK_DROP:

        if msg == "DONE":
            print("Cycle Complete")
            state = STATE_WAIT_PICK
            payload_signal_sent = False

        det1 = find_tag(detections, 1)

        if det1 is not None:
            t = det1.pose_t
            x_cm = meter_to_cm(float(t[0]))
            y_cm = meter_to_cm(float(t[1]))

            distance = math.sqrt(x_cm**2 + y_cm**2)

            if distance <= DIST_THRESHOLD_CM:
                ser.write(b"DROP\n")
                print("ID1 near (0,0) -> DROP sent")
                state = STATE_WAIT_FINAL_DONE

            if current_time - last_target_sent_time >= SEND_INTERVAL:
                command = f"TARGET {x_cm} {y_cm}\n"
                ser.write(command.encode())
                print("Tracking DROP:", command.strip())
                last_target_sent_time = current_time

        else:
            print("Drop tag not visible")

    cv2.imshow("apriltag", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break


cap.release()
ser.close()
cv2.destroyAllWindows()
