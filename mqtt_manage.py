import paho.mqtt.client as mqtt
import json
import threading

class MQTTManager:
    def __init__(self, client_host, client_port=1883, keepalive=60):
        self.cmd = 1
        self.dest_x = 3.0
        self.dest_z = 1.5
        
        self.cmd_semaphore = threading.Semaphore()
        self.dest_x_semaphore = threading.Semaphore()
        self.dest_z_semaphore = threading.Semaphore()

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(client_host, client_port, keepalive)
        self.thread = threading.Thread(target=self.client.loop_forever)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe("/mqtt/scv1/forward")

    def on_message(self, client, userdata, msg):
        received_msg = msg.payload.decode()
        msg_json = json.loads(received_msg)
        command = msg_json["command"]
        coord = msg_json["coord"]
        client.publish("/mqtt/scv1/backward", coord)

        try:
            new_cmd = command
            new_dest_x, new_dest_z = map(float, coord.split(','))
            self.update_cmd_dest_values(new_cmd, new_dest_x, new_dest_z)
            print(f"Received new values: cmd={new_cmd}, dest_x={new_dest_x}, dest_z={new_dest_z}")
            
        except ValueError:
            print("Received invalid message format")

    def update_cmd_dest_values(self, new_cmd, new_dest_x, new_dest_z):
        with self.cmd_semaphore:
            self.cmd = new_cmd

        with self.dest_x_semaphore:
            self.dest_x = new_dest_x

        with self.dest_z_semaphore:
            self.dest_z = new_dest_z

    def start(self):
        self.thread.start()

    def stop(self):
        self.client.disconnect()
        self.thread.join()

    def get_cmd(self):
        with self.cmd_semaphore:
            return self.cmd

    def get_dest(self):
        with self.dest_x_semaphore, self.dest_z_semaphore:
            return self.dest_x, self.dest_z
