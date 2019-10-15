import sys
import time
import socket
import traceback
import selectors
from struct import unpack

class OurConnection:
    def __init__(self):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5000
        self.BUFFER_SIZE = 1024
        self.conn = None
        self.failed_counter = 0
        self.num_floats = 10
        self.packet_size = self.num_floats * 4

        # replay
        self.last_episode = list()
        self.is_replaying = False
        self.last_episode_progress = 0

        self.total_data = bytearray()
        self.initialize_and_bind_server()
        self.reconnect()

    def initialize_and_bind_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.settimeout(0.0)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.TCP_IP, self.TCP_PORT))
        self.server.listen()

    def is_server_running(self):
        return not self.server._closed

    def is_conn_open(self):
        return (self.conn is not None) and (not self.conn._closed)
    
    def close_server(self):
        if not self.is_server_running():
            print("Closing server ignored.  Bad idea.")
            return

        self.server.close()

    def reconnect(self):
        if self.is_conn_open():
            print("Reconnection ignored.  Bad idea.")
            return

        try:
            self.conn, addr = self.server.accept()
            self.conn.settimeout(0.0)
            self.last_episode = list()
            self.last_episode_progress = 0
        except Exception:
            self.failed_counter += 1
            if self.failed_counter % 20 == 0:
                print("BUSY WAITING FOR CONNECTION INITIALIZATION")

    def disconnect(self):
        if not self.is_conn_open():
            print("Disconnection ignored.  Bad idea.")
            return

        self.conn.close()

    def disconnect_if_no_data(self, data):
        if len(data) == 0:
            self.disconnect()
            return True
        return False

    def destroy_all(self):
        self.disconnect()
        self.close_server()

    def __del__(self):
        self.destroy_all()

    def add_data(self, data):
        self.total_data.extend(data)  # removed add here
    
    def can_process(self):
        return len(self.total_data) >= self.packet_size

    def get_and_clear_buffer(self):
        # could cause race condition here on the total_data stuff.
        list_data = list()
        present = True
        while present:
            trimmed_data = self.total_data[:self.packet_size]
            self.last_episode.append(trimmed_data)
            list_data.append(trimmed_data)
            self.total_data = self.total_data[self.packet_size:]
            present = self.can_process()
        return list_data

    def process_entry(self, data):
        format = ">" + "f" * self.num_floats
        unpacked_data = list(unpack(format, data))

        rocket = bpy.data.objects["Rocket"]
        rocket.location = unpacked_data[0:3]
        rocket.rotation_quaternion = unpacked_data[3:7]

        # TODO: FINISH IMPLEMENTING THESE
        # thurst = unpacked_data[7]
        # gimble_x = unpacked_data[8]
        # gimble_y = unpacked_data[9]
    
if __name__ == "__main__":
    OUR_CONNECTION = OurConnection()
    while True:
        assert OUR_CONNECTION.is_server_running() is True
        if not OUR_CONNECTION.is_conn_open():
            OUR_CONNECTION.reconnect()
        if not OUR_CONNECTION.is_conn_open():
            time.sleep(0.1)
            continue
        data = OUR_CONNECTION.conn.recv(OUR_CONNECTION.BUFFER_SIZE)
        print(data)

    print("FINISHED!")
    
