#!/usr/bin/env python
# coding: utf-8


import mediapipe as mp


class LivePersonLocationDetector:
    def __init__(self, focal_length=1000):
        self.focal_length = focal_length
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()

    def process_frame(self, frame):
        # # Convert the BGR frame to RGB
        # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Pose
        results = self.pose.process(frame)

        h, w, _ = frame.shape
        image_center = (w // 2, h // 2)

        if results.pose_landmarks:
            left_shoulder = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            shoulder_width_pixels = abs(left_shoulder.x * w - right_shoulder.x * w)

            person_center_x = (left_shoulder.x + right_shoulder.x) * w / 2

            real_z_distance = (0.4 * self.focal_length) / shoulder_width_pixels
            X_real = (person_center_x - image_center[0]) * real_z_distance / self.focal_length

            return {"X": X_real, "Z": real_z_distance}

        return None

    def close(self):
        self.pose.close()


# # Example usage for a live video stream (or video file)
# def main():  # Use 0 for webcam
#     video_detector = LivePersonLocationDetector()
#     cap = cv2.VideoCapture(video_source)

#     if not cap.isOpened():
#         print("Error: Could not open video source.")
#         return

#     while cap.isOpened():
#         ret, frame = cap.read()
#         if not ret:
#             break

#         location = video_detector.process_frame(frame)
#         if location:
#             print(f"Live Location: X = {location['X']:.2f} m, Z = {location['Z']:.2f} m")
#             # Integrate with your robot's navigation system

#         # Optional: Show the frame for debugging purposes
#         cv2.imshow('Frame', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     video_detector.close()
#     cv2.destroyAllWindows()


# if __name__ == "__main__":
#     main()
