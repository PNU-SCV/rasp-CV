import serial
import struct

class UARTCommunication:
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ESP32SendData_format = '<BfffB'  
        self.ESP32RecvData_format = '<BffB'   

    def calculate_crc(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF  
        return crc

    def send_data(self, cmd, dest_x, dest_z, angle):
        data = struct.pack(self.ESP32SendData_format[:-1], cmd, dest_x, dest_z, angle)  
        crc = self.calculate_crc(data)
        data_with_crc = struct.pack(self.ESP32SendData_format, cmd, dest_x, dest_z, angle, crc)
        self.ser.write(data_with_crc)
        print(f"Sent: cmd={cmd}, dest_x={dest_x}, dest_z={dest_z}, angle={angle}, crc={crc}")
    
    def receive_data(self):
        data_size = struct.calcsize(self.ESP32RecvData_format)
        data = self.ser.read(data_size)  
        if len(data) == data_size:
            received_crc = data[-1]
            data_without_crc = data[:-1]
            calculated_crc = self.calculate_crc(data_without_crc)
            
            if received_crc == calculated_crc:
                stat, loc_x, loc_z = struct.unpack(self.ESP32RecvData_format[:-1], data_without_crc)
                print(f"Received: stat={stat}, loc_x={loc_x}, loc_z={loc_z}")
                return stat, loc_x, loc_z
            else:
                print("CRC not matched")
            
        
        self.ser.reset_input_buffer()
        return None

    def close(self):
        self.ser.close()
