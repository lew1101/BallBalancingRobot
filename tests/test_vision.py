import cv2

from picamera2 import Picamera2  # type:ignore

from src.vision import *
from src.constants import *


def test_vision():
    picam2 = Picamera2()

    try:
        picam2 = Picamera2()
        sensor_w, sensor_h = picam2.sensor_resolution
        config = picam2.create_preview_configuration(main={
            "size": (sensor_w // 4, sensor_h // 4),
            'format': 'RGB888'
        },)
        picam2.configure(config)
        picam2.start()

        cv2.startWindowThread()

        while True:
            request = picam2.capture_request()
            frame = request.make_array("main")
            request.release()

            if frame is None:
                break
            # Downsample frame to reduce processing time
            square_crop = frame[::2, ::2]  # downsample by 2x
            frame = cv2.resize(square_crop, OUTPUT_SIZE, interpolation=cv2.INTER_NEAREST)

            color, cv_centre, radius = getBallPos(frame, debug=False)

            # draw current setpoint
            cvSetX, cvSetY = robotToCvCoords((0.0, 0.0))
            cv2.circle(frame, (int(cvSetX), int(cvSetY)), radius=4, color=(0, 0, 255), thickness=-1)

            if cv_centre is None:  # no ball detected
                cv2.imshow("Preview", frame)
                continue

            # convert cv coordinates to robot coordinates
            ballX, ballY = cvToRobotCoords(cv_centre)

            # draw ball position
            cx, cy = cv_centre
            cv2.circle(frame, (int(cx), int(cy)), int(radius), COLOR_BGR[color], 2)
            cv2.putText(frame, f"({ballX:.2f}, {ballY:.2f})", (int(cx) - 20, int(cy) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_BGR[color], 2)
            cv2.imshow("Preview", frame)

            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        picam2.stop()

        cv2.destroyAllWindows()
