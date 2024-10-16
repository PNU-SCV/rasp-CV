import cv2
import numpy as np
from picamera2 import Picamera2
from aruco_detector import ArucoDetector
from uart_communication import UARTCommunication
from utils import find_shortest_path_astar, simplify_path, real_to_grid, grid_to_real
from grid import create_grid
import paho.mqtt.client as mqtt
import json
import time
import threading
import math

WITHOUT_MQTT = False

cmd = 1
dest_x = 3.0
dest_z = 3.5
angle = 0.0

cell_size_cm = 80  
tolerance_m = 0.2  
temp_obstacle_duration = 5  

max_x = 7.3  
max_z = 7.3  

bitmap = None
grid_size_x = None
grid_size_z = None
obstacles = [[0,0],[0,1],[0,2],[0,3],[0,6],[0,7],[0,8],
             [1,0],[1,1],[1,2],[1,3],[1,6],[1,7],[1,8],
             [2,3],[2,6],[2,7],[2,8],
             [3,3],[3,6],[3,7],[3,8],
             [4,6],[4,7],[4,8],
             [5,6],[5,7],[5,8],
             [6,3],
             [7,3],[7,6],[7,7],[7,8],
             [8,3],[8,6],[8,7],[8,8]]

temp_obstacles = []

camera_matrix = np.array([[829.0928006935812, 0.0, 320.51025314582233],
                          [0.0, 829.3696919624016, 314.76661310314887],
                          [0.0, 0.0, 1.0]])

dist_coeffs = np.array([0.17667058749632775, -1.0737869272298604, 
                        0.0021937583876734733, 0.0010397571854110412, 
                        1.40169723543808])

mqtt_stat = 0
mqtt_loc_x, mqtt_loc_z = 0.0, 0.0

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("/mqtt/scv1/forward")

def on_message(client, userdata, msg):
    global cmd, dest_x, dest_z
    received_msg = msg.payload.decode()
    msg_json = json.loads(received_msg)
    new_cmd = msg_json.get("command")
    coord = msg_json.get("coord", "0,0")
    new_dest_x, new_dest_z = map(float, coord.split(','))
    
    cmd = new_cmd
    dest_x = new_dest_x
    dest_z = new_dest_z
    print(f"Received new values: cmd={new_cmd}, dest_x={new_dest_x}, dest_z={new_dest_z}")

def manage_temp_obstacles():
    global temp_obstacles, bitmap
    current_time = time.time()
    new_temp_obstacles = []

    for grid_x, grid_z, t in temp_obstacles:
        if current_time - t < temp_obstacle_duration:
            new_temp_obstacles.append((grid_x, grid_z, t))
        else:
            bitmap[grid_z, grid_x] = 1  
    
    temp_obstacles = new_temp_obstacles

def send_status_mqtt(client, mqtt_stat_lock, mqtt_loc_lock):
    global mqtt_stat, mqtt_loc_x, mqtt_loc_z
    
    temp_stat = 0
    temp_loc_x = 0
    temp_loc_z = 0
    
    while True:
        with mqtt_stat_lock:
            temp_stat = mqtt_stat
            
        with mqtt_loc_lock:
            temp_loc_x, temp_loc_z = mqtt_loc_x, mqtt_loc_z
        
        status_data = {
            "id":"scv1",
            "status": temp_stat,
            "loc_x": temp_loc_x,
            "loc_z": temp_loc_z
        }
        client.publish("/mqtt/scv1/backward", json.dumps(status_data))
        print(f"Sent status: {status_data}")
        
        time.sleep(1)

def update_angle(picam2, aruco_detector, angle_lock):
    global angle
    while True:
        img = picam2.capture_array()
        detected_angle = aruco_detector.detect_marker_angle(img)

        if detected_angle is not None:
            with angle_lock:
                angle = detected_angle
        
        time.sleep(0.01)

def main(mqtt_client):
    global cmd, dest_x, dest_z, bitmap, grid_size_x, grid_size_z, temp_obstacles, mqtt_stat, mqtt_loc_x, mqtt_loc_z
    mqtt_stat, loc_x, loc_z = 0, 0, 0
    
    bitmap, grid_size_x, grid_size_z = create_grid(max_x, max_z, cell_size_cm, obstacles)
    print(bitmap)
    
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.start()

    aruco_detector = ArucoDetector(camera_matrix, dist_coeffs)
    uart_comm = UARTCommunication(port='/dev/serial0', baudrate=115200, timeout=0.01)

    path = []
    angle_lock = threading.Lock()
    state_lock = threading.Lock()
    mqtt_stat_lock = threading.Lock()
    mqtt_loc_lock = threading.Lock()

    angle_thread = threading.Thread(target=update_angle, args=(picam2, aruco_detector, angle_lock))
    angle_thread.start()
    
    mqtt_send_thread = threading.Thread(target=send_status_mqtt, args=(mqtt_client, mqtt_stat_lock, mqtt_loc_lock))
    mqtt_send_thread.start()
    
    try:
        while True:
            with state_lock:
                current_cmd = cmd
            
            if current_cmd == 0:
                uart_comm.send_data(current_cmd, 0.0, 0.0, 0.0)
        
                time.sleep(0.1)
                continue
            
            recv_data = uart_comm.receive_data()
            
            if not recv_data:
                continue
            
            stat, loc_x, loc_z = recv_data
            
            with mqtt_loc_lock:
                mqtt_loc_x, mqtt_loc_z = loc_x, loc_z
            
            if WITHOUT_MQTT == False:
                with mqtt_stat_lock:
                    mqtt_stat = 0
            
            manage_temp_obstacles()

            if stat != 0:  
                current_grid_x, current_grid_z = real_to_grid(loc_x, loc_z, cell_size_cm)
                center_x, center_z = grid_to_real(current_grid_x, current_grid_z, cell_size_cm)
                distance_to_center = math.sqrt((loc_x - center_x) ** 2 + (loc_z - center_z) ** 2)

                if distance_to_center < tolerance_m:
                    forward_x = current_grid_x + int(math.cos(math.radians(angle)))
                    forward_z = current_grid_z + int(math.sin(math.radians(angle)))
                    temp_obstacles.append((forward_x, forward_z, time.time()))
                    bitmap[forward_z, forward_x] = 0  
                else:
                    temp_obstacles.append((current_grid_x, current_grid_z, time.time()))
                    bitmap[current_grid_z, current_grid_x] = 0
                    
                continue
                
            with state_lock:
                start_point = real_to_grid(loc_x, loc_z, cell_size_cm)
                end_point = real_to_grid(dest_x, dest_z, cell_size_cm)
                
            print(start_point, end_point)
            

            if start_point and end_point:
                if not path or end_point != path[-1]:
                    shortest_path = find_shortest_path_astar(bitmap, start_point, end_point)
                    path = simplify_path(shortest_path)
                
            print(path)

            if not path:
                continue

            next_point = path[0]
            
            center_x, center_z = grid_to_real(next_point[0], next_point[1], cell_size_cm)
            print(center_x, center_z)

            distance_to_center = math.sqrt((loc_x - center_x) ** 2 + (loc_z - center_z) ** 2)
            if distance_to_center < tolerance_m:
                path.pop(0)
                with state_lock:
                    cmd = 0 if not path else 1
                    
            if not path:
                with mqtt_stat_lock:
                    mqtt_stat = 5

            with angle_lock:
                current_angle = angle

            with state_lock:
                current_cmd = cmd
                
            print(current_angle)
                
            uart_comm.send_data(current_cmd, center_x, center_z, current_angle)
                
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("STOPPED")
    finally:
        uart_comm.close()
        cv2.destroyAllWindows()
        angle_thread.join()

if __name__ == "__main__":
    mqtt_client = 1
    if WITHOUT_MQTT == False:
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect("192.168.0.5", 1883, 60)
        
        mqtt_thread = threading.Thread(target=mqtt_client.loop_forever)
        mqtt_thread.start()
    else:
        mqtt_client = 0

    main(mqtt_client)

    mqtt_client.loop_stop()
    mqtt_thread.join()
