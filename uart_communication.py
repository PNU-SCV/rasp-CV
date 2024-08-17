import serial
import struct

class UARTCommunication:
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ESP32SendData_format = 'Bfff'
        self.ESP32RecvData_format = 'Bff'

    def send_data(self, cmd, dest_x, dest_z, angle):
        data = struct.pack(self.ESP32SendData_format, cmd, dest_x, dest_z, angle)
        self.ser.write(data)
        print(f"Sent: cmd={cmd}, dest_x={dest_x}, dest_z={dest_z}, angle={angle}")
    
    def receive_data(self):
        data_size = struct.calcsize(self.ESP32RecvData_format)
        data = self.ser.read(data_size)
        if len(data) == data_size:
            stat, loc_x, loc_z = struct.unpack(self.ESP32RecvData_format, data)
            print(f"Received: stat={stat}, loc_x={loc_x}, loc_z={loc_z}")
            return stat, loc_x, loc_z
        else:
            print("No data received or incomplete data")
            return None

    def close(self):
        self.ser.close()
