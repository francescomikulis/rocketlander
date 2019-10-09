import bpy
import sys
import time
import socket
import traceback
from mathutils import Vector


# /Applications/Blender.app/Contents/Resources/2.80/python/bin/python3.7m -m pip install ipdb
# /Applications/Blender.app/Contents/Resources/2.80/python/bin/python3.7m -m pip install ipdb

class OurConnection:
    def __init__(self):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5000
        self.BUFFER_SIZE = 1024
        self.initialize_and_bind_server()
        self.conn = None
        self.reconnect()

    def initialize_and_bind_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.settimeout(0.05)
        self.server.bind((self.TCP_IP, self.TCP_PORT))
        self.server.listen()

    def is_server_running(self):
        return not self.server._closed

    def is_conn_open(self):
        return (self.conn is not None) and (not self.conn._closed)

    def close_server(self):
        if not self.is_server_running():
            print("Closing server ignored.  Bad idea.")

        self.server.close()

    def reconnect(self):
        if self.is_conn_open():
            print("Reconnection ignored.  Bad idea.")
            return

        try:
            self.conn, addr = self.server.accept()
        except Exception:
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
        OUR_CONNECTION.disconnect()
        OUR_CONNECTION.close_server()

    def __del__(self):
        self.destroy_all()


OUR_CONNECTION = OurConnection()

class ModalTimerOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"

    _timer = None

    def updateRocket(self, data):
        rocket = bpy.data.objects["Rocket"]
        vec = Vector((0.1, 0.1, 0.1))
        rocket.location = rocket.location + vec

    def modal(self, context, event):
        try:
            return self.real_modal(context, event)
        except KeyboardInterrupt:
            self.cancel(context)
            return {'CANCELLED'}

    def real_modal(self, context, event):
        if event.type in {'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            assert OUR_CONNECTION.is_server_running() is True

            if not OUR_CONNECTION.is_conn_open():
                OUR_CONNECTION.reconnect()
                return {'PASS_THROUGH'}

            assert OUR_CONNECTION.is_conn_open() is True

            try:
                data = OUR_CONNECTION.conn.recv(OUR_CONNECTION.BUFFER_SIZE)
                decoded_data = data.decode('utf-8')
                print("received data:", decoded_data)

                needed_to_reconnect = OUR_CONNECTION.disconnect_if_no_data(decoded_data)
                if not needed_to_reconnect:
                    self.updateRocket(data)

            except Exception as e:
                print("FAILURE EXCEPTION")
                print(e)
                return {'PASS_THROUGH'}

        return {'PASS_THROUGH'}


    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
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
