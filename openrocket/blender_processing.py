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
        self.BUFFER_SIZE = 256
        self.conn = None
        self.failed_counter = 0
        self.death_penalty = 0
        self.end_marker = b'*'
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
        return self.end_marker in self.total_data

    def get_and_clear_buffer(self):
        # could cause race condition here on the total_data stuff.
        list_data = list()
        present = True
        mark = self.end_marker
        while present:
            loc = self.total_data.find(mark, 63)
            trimmed_data = self.total_data[:loc]
            if len(trimmed_data) == 63:
                list_data.append(trimmed_data)
            self.total_data = self.total_data[loc + len(mark):]
            present = mark in self.total_data
        return list_data

    def process_entry(self, data):
        map = {'x':0, 'y':1, 'z':2, 'W':3, 'X':4, 'Y':5, 'Z':6}
        format = ">" + "cd" * 7
        x, x_val, y, y_val, z, z_val, W, W_val, X, X_val, Y, Y_val, Z, Z_val = unpack(format, data)

        rocket = bpy.data.objects["Rocket"]

        rocket.location[0] = x_val
        rocket.location[1] = y_val
        rocket.location[2] = z_val

        rocket.rotation_quaternion[0] = W_val
        rocket.rotation_quaternion[1] = X_val
        rocket.rotation_quaternion[2] = Y_val
        rocket.rotation_quaternion[3] = Z_val

    def increment_death_penalty(self):
        self.death_penalty += 1

    def is_destined_to_death(self):
        return self.death_penalty == 10000




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

    def modal(self, context, event):
        if event.type in {'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            assert OUR_CONNECTION.is_server_running() is True

            if not OUR_CONNECTION.is_conn_open():
                OUR_CONNECTION.reconnect()
                return {'PASS_THROUGH'}

            assert OUR_CONNECTION.is_conn_open() is True

            num_attempts = 2
            curr_attempts = 0
            while (curr_attempts < num_attempts):
                try:
                    data = OUR_CONNECTION.conn.recv(OUR_CONNECTION.BUFFER_SIZE)

                    needed_to_kill_because_done = OUR_CONNECTION.disconnect_if_no_data(data)

                    need_to_recurse = False
                    if (len(data) == OUR_CONNECTION.BUFFER_SIZE):
                        data = data[OUR_CONNECTION.BUFFER_SIZE - 64:]
                        need_to_recurse = True

                    OUR_CONNECTION.add_data(data)

                    if OUR_CONNECTION.can_process():
                        self.updateRocket()

                    if need_to_recurse:
                        return self.modal(context, event)

                    if needed_to_kill_because_done:
                        OUR_CONNECTION.increment_death_penalty()

                    if OUR_CONNECTION.is_destined_to_death():
                        self.cancel(context)
                        return {'CANCELLED'}

                    curr_attempts = num_attempts + 1

                except Exception as e:
                    curr_attempts += 1
                    #print("FAILURE EXCEPTION")
                    #print(e)
                    #print(traceback.print_exc())
                    pass

        return {'PASS_THROUGH'}


    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        OUR_CONNECTION.destroy_all()



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
