#!/usr/bin/env python3

import socket
import mediapipe as mp
import cv2

HOST = '10.42.0.1'  # The server's hostname or IP address
PORT = 60001      # The port used by the server

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

cap = cv2.VideoCapture(0)
image_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
image_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

MAX_CONTROL_PIXELS = 350
MIN_CONTROL_PIXELS = -350

last_finger_positions = (0, 0, 0, 0)
last_positions = (0, 0, 0, 0)
last_engaged = False
locked_origin = (0, 0, 0, 0)
last_valid_positions = (0, 0, 0, 0)


def distance(x1 : int, y1 : int, x2 : int, y2 : int) -> int:
    return int((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) ** 0.5)

def in_bounds(x : int, y : int, start : tuple, end : tuple) -> bool:
    return start[0] < x < end[0] and start[1] < y < end[1]

def in_range(x : int, maximum : int, minimum : int) -> int:
    if x < minimum:
        return minimum
    if x > maximum:
        return maximum
    return x


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    # Initiate holistic model
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            left_fist = False
            right_fist = False
            valid = False

            ret, frame = cap.read()

            # Recolor Feed
            # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = frame
            # Make Detections
            results = holistic.process(image)
            # print(results.face_landmarks)

            # face_landmarks, pose_landmarks, left_hand_landmarks, right_hand_landmarks

            # Recolor image back to BGR for rendering
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Get coordinates of hand locations if possible
            if results.pose_landmarks is not None:
                # Get pixel positions
                left_x = results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_INDEX].x * image_width
                left_y = results.pose_landmarks.landmark[mp_holistic.PoseLandmark.LEFT_INDEX].y * image_height
                right_x = results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].x * image_width
                right_y = results.pose_landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_INDEX].y * image_height

                current_positions = (left_x, left_y, right_x, right_y)
                valid = True
                for i, position in enumerate(last_positions):
                    if abs(position - current_positions[i]) > 80:
                        valid = False

                last_positions = current_positions

                # Get coordinates of index finger on both hands if possible
                if (results.right_hand_landmarks is not None) and (results.left_hand_landmarks is not None):
                    left_finger_x = results.left_hand_landmarks.landmark[
                                        mp_holistic.HandLandmark.INDEX_FINGER_TIP].x * image_width
                    left_finger_y = results.left_hand_landmarks.landmark[
                                        mp_holistic.HandLandmark.INDEX_FINGER_TIP].y * image_height
                    right_finger_x = results.right_hand_landmarks.landmark[
                                         mp_holistic.HandLandmark.INDEX_FINGER_TIP].x * image_width
                    right_finger_y = results.right_hand_landmarks.landmark[
                                         mp_holistic.HandLandmark.INDEX_FINGER_TIP].y * image_height

                    current_finger_positions = (left_finger_x, left_finger_y, right_finger_x, right_finger_y)
                    for i, position in enumerate(last_finger_positions):
                        if abs(position - current_finger_positions[i]) > 50:
                            valid = False

                    last_finger_positions = current_finger_positions

            # Fist detection
            if valid and ((results.right_hand_landmarks is not None) and (results.left_hand_landmarks is not None)):
                if distance(left_finger_x, left_finger_y, left_x, left_y) < 100:
                    left_fist = True
                if distance(right_finger_x, right_finger_y, right_x, right_y) < 100:
                    right_fist = True

                # Draw circles at hand locations
                if valid:
                    if left_fist:
                        cv2.circle(image, (int(left_x), int(left_y)), 30, (0, 255, 0), -1)
                    else:
                        cv2.circle(image, (int(left_x), int(left_y)), 30, (0, 0, 255), -1)

                    if right_fist:
                        cv2.circle(image, (int(right_x), int(right_y)), 30, (0, 255, 0), -1)
                    else:
                        cv2.circle(image, (int(right_x), int(right_y)), 30, (0, 0, 255), -1)

                    # Determine if engaged
                    if left_fist and right_fist:
                        cv2.circle(image, (100, 100), 30, (0, 255, 0), -1)
                        if last_engaged is False:
                            locked_origin = (left_x, left_y, right_x, right_y)
                    else:
                        cv2.circle(image, (int(right_x), int(right_y)), 30, (0, 0, 255), -1)

            # Draw disengagement bounds
            right_bound_start = (0, 0)
            right_bound_end = (int(image_width / 5), int(image_height))
            left_bound_start = (int(image_width - (image_width / 5)), 0)
            left_bound_end = (int(image_width), int(image_height))
            cv2.rectangle(image, right_bound_start, right_bound_end, (0, 255, 0), 3)
            cv2.rectangle(image, left_bound_start, left_bound_end, (0, 255, 0), 3)

            # Determine if engaged
            if valid:
                current_engaged = left_fist and right_fist
                if (not last_engaged) and current_engaged:
                    locked_origin = (left_x, left_y, right_x, right_y)
                if in_bounds(left_x, left_y, left_bound_start, left_bound_end) and in_bounds(right_x, right_y,
                                                                                             right_bound_start,
                                                                                             right_bound_end):
                    if (not left_fist) and (not right_fist):
                        last_engaged = False
                if current_engaged:
                    last_engaged = True
                last_valid_positions = (left_x, left_y, right_x, right_y)

            if last_engaged:
                cv2.circle(image, (100, 100), 30, (0, 255, 0), -1)
                # determine & draw controls
                # left
                cv2.rectangle(image, (int(locked_origin[0]), int(locked_origin[1])),
                              (int(last_valid_positions[0]), int(last_valid_positions[1])), (0, 0, 255), 3)
                # right
                cv2.rectangle(image, (int(locked_origin[2]), int(locked_origin[3])),
                              (int(last_valid_positions[2]), int(last_valid_positions[3])), (0, 0, 255), 3)
                left_control = in_range(last_valid_positions[1] - locked_origin[1], MAX_CONTROL_PIXELS, MIN_CONTROL_PIXELS)
                right_control = in_range(last_valid_positions[3] - locked_origin[3], MAX_CONTROL_PIXELS, MIN_CONTROL_PIXELS)

                left_control = int((left_control / -MAX_CONTROL_PIXELS) * 10) / 10
                right_control = int((right_control / -MAX_CONTROL_PIXELS) * 10) / 10

                # Send updated information to server
                s.sendall(bytes(str(left_control) + ',' + str(right_control), 'utf-8'))

            else:
                cv2.circle(image, (100, 100), 30, (0, 0, 255), -1)
                s.sendall(bytes('0.0,0.0', 'utf-8'))

            # Flip image for display
            image = cv2.flip(image, 1)
            cv2.imshow('Raw Webcam Feed', image)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

cap.release()
cv2.destroyAllWindows()