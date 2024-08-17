import cv2
from picamera2 import Picamera2
from aruco_detector import ArucoDetector
from uart_communication import UARTCommunication
import time

cmd = 1           # 임의의 값
dest_x = 3.0      # 임의의 값
dest_z = 1.5      # 임의의 값
angle = 0.0       # 초기 각도 값

camera_matrix = np.array([[829.0928006935812, 0.0, 320.51025314582233],
                          [0.0, 829.3696919624016, 314.76661310314887],
                          [0.0, 0.0, 1.0]])

dist_coeffs = np.array([0.17667058749632775, -1.0737869272298604, 
                        0.0021937583876734733, 0.0010397571854110412, 
                        1.40169723543808])

def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.start()

    aruco_detector = ArucoDetector(camera_matrix, dist_coeffs)
    uart_comm = UARTCommunication(port='/dev/serial0', baudrate=115200, timeout=0.1)

    try:
        while True:
            img = picam2.capture_array()
            
            detected_angle = aruco_detector.detect_marker_angle(img)
            
            if detected_angle is not None:
                global angle
                angle = detected_angle
                cv2.putText(img, f'Angle: {angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # UART로 데이터 전송
                uart_comm.send_data(cmd, dest_x, dest_z, angle)
            
            cv2.imshow("Aruco Marker Detection", img)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("STOPPED")
    finally:
        uart_comm.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
