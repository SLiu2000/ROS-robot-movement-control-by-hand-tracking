#!/usr/bin/env python3

import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

cap = cv2.VideoCapture(0)
image_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
image_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

last_finger_positions = (0, 0, 0, 0)
last_positions = (0, 0, 0, 0)

def distance(x1 : int, y1 : int, x2 : int, y2 : int) -> int:
    return int((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) ** 0.5)

with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        left_fist = False
        right_fist = False
        valid = False

        success, image = cap.read()
        results = holistic.process(image)

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

        mp_drawing.draw_landmarks(
            image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
        mp_drawing.draw_landmarks(
            image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

        # Flip image for display
        image = cv2.flip(image, 1)
        cv2.imshow('Raw Webcam Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()