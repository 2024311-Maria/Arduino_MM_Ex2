import argparse
import math
import time

import cv2
import serial

try:
    from mediapipe.python.solutions import drawing_utils
    from mediapipe.python.solutions import pose as pose_module
except ImportError as error:
    raise ImportError(
        "MediaPipe Pose is unavailable. Install the project dependencies with "
        "`python -m pip install -r requirements.txt`."
    ) from error


def calculate_angle(point_a, vertex, point_c, frame_width, frame_height):
    """Return the 0-180 degree angle at vertex in image coordinates."""
    first = (
        (point_a.x - vertex.x) * frame_width,
        (point_a.y - vertex.y) * frame_height,
    )
    second = (
        (point_c.x - vertex.x) * frame_width,
        (point_c.y - vertex.y) * frame_height,
    )
    dot = first[0] * second[0] + first[1] * second[1]
    first_length = (first[0] ** 2 + first[1] ** 2) ** 0.5
    second_length = (second[0] ** 2 + second[1] ** 2) ** 0.5
    if first_length == 0 or second_length == 0:
        return None

    cosine = max(-1.0, min(1.0, dot / (first_length * second_length)))
    return round(math.degrees(math.acos(cosine)))


def main():
    parser = argparse.ArgumentParser(
        description="Track right shoulder and elbow angles and control two servos."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Arduino serial port (for example COM3 on Windows or /dev/ttyACM0)",
    )
    parser.add_argument(
        "--camera", type=int, default=0, help="webcam index (default: 0)"
    )
    args = parser.parse_args()

    right_shoulder = pose_module.PoseLandmark.RIGHT_SHOULDER.value
    right_elbow = pose_module.PoseLandmark.RIGHT_ELBOW.value
    right_wrist = pose_module.PoseLandmark.RIGHT_WRIST.value
    right_hip = pose_module.PoseLandmark.RIGHT_HIP.value

    with serial.Serial(args.port, 115200, timeout=1) as arduino:
        time.sleep(2)
        camera = cv2.VideoCapture(args.camera)
        if not camera.isOpened():
            camera.release()
            raise RuntimeError(f"Could not open camera {args.camera}")

        try:
            with pose_module.Pose(
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            ) as pose:
                while True:
                    success, frame = camera.read()
                    if not success:
                        raise RuntimeError("Could not read a frame from the camera")

                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    result = pose.process(rgb_frame)
                    if result.pose_landmarks:
                        landmarks = result.pose_landmarks.landmark
                        indices = (
                            right_shoulder,
                            right_elbow,
                            right_wrist,
                            right_hip,
                        )
                        if all(landmarks[index].visibility >= 0.5 for index in indices):
                            shoulder_angle = calculate_angle(
                                landmarks[right_hip],
                                landmarks[right_shoulder],
                                landmarks[right_elbow],
                                frame.shape[1],
                                frame.shape[0],
                            )
                            elbow_angle = calculate_angle(
                                landmarks[right_shoulder],
                                landmarks[right_elbow],
                                landmarks[right_wrist],
                                frame.shape[1],
                                frame.shape[0],
                            )

                            if shoulder_angle is not None and elbow_angle is not None:
                                arduino.write(
                                    f"{shoulder_angle},{elbow_angle}\n".encode("ascii")
                                )
                                cv2.putText(
                                    frame,
                                    f"Shoulder: {shoulder_angle}  Elbow: {elbow_angle}",
                                    (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (0, 255, 0),
                                    2,
                                )

                        drawing_utils.draw_landmarks(
                            frame, result.pose_landmarks, pose_module.POSE_CONNECTIONS
                        )

                    cv2.imshow("Right arm servo control (Q to quit)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        finally:
            camera.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
