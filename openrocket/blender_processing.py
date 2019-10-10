import bpy
import sys
import time
import socket
import traceback
import selectors
from struct import unpack
from mathutils import Vector


# /Applications/Blender.app/Contents/Resources/2.80/python/bin/python3.7m -m pip install ipdb
# /Applications/Blender.app/Contents/Resources/2.80/python/bin/python3.7m -m pip install ipdb

class OurConnection:
    def __init__(self):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5000
        self.BUFFER_SIZE = 1024
        self.conn = None
        self.failed_counter = 0
        self.packet_size = 58

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
        format = ">" + "c" + "d" * 3 + "c" + "d" * 4
        pos, x_val, y_val, z_val, ori, W_val, X_val, Y_val, Z_val = unpack(format, data)

        rocket = bpy.data.objects["Rocket"]

        rocket.location[0] = x_val
        rocket.location[1] = y_val
        rocket.location[2] = z_val

        rocket.rotation_quaternion[0] = W_val
        rocket.rotation_quaternion[1] = X_val
        rocket.rotation_quaternion[2] = Y_val
        rocket.rotation_quaternion[3] = Z_val




OUR_CONNECTION = OurConnection()

class ModalTimerOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"

    _timer = None

    def updateRocket(self):
        assert OUR_CONNECTION.can_process()
        data = OUR_CONNECTION.get_and_clear_buffer()
        for entry in data:
            OUR_CONNECTION.process_entry(entry)

    def replay_logic(self):
        if OUR_CONNECTION.is_replaying:
            receiving_live_data = OUR_CONNECTION.is_server_running() and OUR_CONNECTION.is_conn_open()

            if not receiving_live_data and OUR_CONNECTION.last_episode_progress < len(OUR_CONNECTION.last_episode):
                OUR_CONNECTION.process_entry(OUR_CONNECTION.last_episode[OUR_CONNECTION.last_episode_progress])
                OUR_CONNECTION.last_episode_progress += 1
                return {'PASS_THROUGH'}
            else:
                OUR_CONNECTION.is_replaying = False
        return None

    def get_data_and_process_logic(self):
        assert OUR_CONNECTION.is_conn_open() is True
        OUR_CONNECTION.is_replaying = False

        try:
            data = OUR_CONNECTION.conn.recv(OUR_CONNECTION.BUFFER_SIZE)
            OUR_CONNECTION.disconnect_if_no_data(data)
            OUR_CONNECTION.add_data(data)

            if OUR_CONNECTION.can_process():
                self.updateRocket()
        except Exception as e:
            #print("FAILURE EXCEPTION")
            #print(e)
            #print(traceback.print_exc())
            pass

    def modal(self, context, event):
        if event.type in {'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'LEFT_ALT':
            OUR_CONNECTION.last_episode_progress = 0
            OUR_CONNECTION.is_replaying = not OUR_CONNECTION.is_replaying
            print("Modifying replay status")
            return {'PASS_THROUGH'}


        if event.type == 'TIMER':
            assert OUR_CONNECTION.is_server_running() is True

            if not OUR_CONNECTION.is_conn_open():
                OUR_CONNECTION.reconnect()

            replaying = self.replay_logic()
            if replaying is not None:
                return replaying

            if not OUR_CONNECTION.is_conn_open():
                return {'PASS_THROUGH'}

            self.get_data_and_process_logic()

        return {'PASS_THROUGH'}


    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        OUR_CONNECTION.destroy_all()
        wm = context.window_manager
        wm.event_timer_remove(self._timer)



def register():
    bpy.utils.register_class(ModalTimerOperator)


def unregister():
    bpy.utils.unregister_class(ModalTimerOperator)


if __name__ == "__main__":
    try:
        register()
        bpy.ops.wm.modal_timer_operator()
    except:
        OUR_CONNECTION.destroy_all()
