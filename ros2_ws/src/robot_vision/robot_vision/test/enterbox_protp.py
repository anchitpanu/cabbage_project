import cv2
import time
from ultralytics import YOLO

model = YOLO(r"C:\Users\HP\OneDrive\เดสก์ท็อป\test\best.pt")

cap = cv2.VideoCapture(0)

STATE = "START_CURVE"

curve_start = time.time()
final_start = None

TARGET_X = 530
FINAL_FORWARD_TIME = 1.6

lost_counter = 0
tag_seen = False

prev_center = None
last_center = None

while True:

    ret, frame = cap.read()
    if not ret:
        break

    action_text = "NONE"

    results = model(frame, conf=0.35)

    tag_detected = False
    center_x = None

    boxes = results[0].boxes

    # -------------------------
    # เลือก TAG BOX ที่เล็กสุด
    # -------------------------

    if boxes is not None and len(boxes) > 0:

        smallest_area = 999999999
        best_box = None

        for box in boxes.xyxy:

            x1,y1,x2,y2 = box

            area = float((x2-x1)*(y2-y1))

            if area < 60000 and area < smallest_area:

                smallest_area = area
                best_box = box


        if best_box is not None:

            x1,y1,x2,y2 = best_box

            center_x = float((x1+x2)/2)

            tag_detected = True
            tag_seen = True
            lost_counter = 0

        else:

            lost_counter += 1

    else:

        lost_counter += 1


    # -------------------------
    # HOLD DETECTION
    # -------------------------

    if tag_detected:

        last_center = center_x

    elif last_center is not None:

        center_x = last_center


    # -------------------------
    # SMOOTH CENTER
    # -------------------------

    if center_x is not None and prev_center is not None:

        center_x = 0.7*prev_center + 0.3*center_x

    prev_center = center_x


    # -------------------------
    # STATE MACHINE
    # -------------------------

    if STATE == "START_CURVE":

        action_text = "CURVE RIGHT"

        if time.time() - curve_start > 2.5:
            STATE = "SEARCH_TAG"


    elif STATE == "SEARCH_TAG":

        if not tag_detected:

            action_text = "ROTATE LEFT"

        else:

            STATE = "ALIGN_TAG"


    elif STATE == "ALIGN_TAG":

        if center_x is None:

            STATE = "SEARCH_TAG"

        else:

            error = center_x - TARGET_X

            if abs(error) < 5:

                STATE = "APPROACH_TAG"

            elif error > 0:

                action_text = "TURN RIGHT"

            else:

                action_text = "TURN LEFT"


    elif STATE == "APPROACH_TAG":

        if tag_detected:

            error = center_x - TARGET_X

            if abs(error) > 20:

                STATE = "ALIGN_TAG"

            else:

                if abs(error) < 8:
                    action_text = "FORWARD"

                elif error > 0:
                    action_text = "FORWARD_RIGHT"

                else:
                    action_text = "FORWARD_LEFT"

        else:

            if tag_seen and lost_counter > 5:

                STATE = "FINAL_FORWARD"
                final_start = time.time()

            else:

                action_text = "TAG LOST"


    elif STATE == "FINAL_FORWARD":

        action_text = "FORWARD"

        if time.time() - final_start > FINAL_FORWARD_TIME:

            STATE = "STOP"


    elif STATE == "STOP":

        action_text = "STOP"


    # -------------------------
    # แสดงผล
    # -------------------------

    annotated = results[0].plot()

    cv2.putText(
        annotated,
        f"STATE: {STATE}",
        (30,40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0,255,0),
        2
    )

    cv2.putText(
        annotated,
        f"ACTION: {action_text}",
        (30,80),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0,255,255),
        2
    )

    if center_x is not None:

        cv2.line(
            annotated,
            (int(center_x),0),
            (int(center_x),480),
            (0,0,255),
            2
        )

    cv2.line(
        annotated,
        (TARGET_X,0),
        (TARGET_X,480),
        (255,0,0),
        2
    )

    cv2.imshow("YOLO Navigation Test", annotated)

    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()