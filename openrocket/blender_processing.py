import bpy
import sys
import time
import socket
from mathutils import Vector

timeout = 1

TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024
global_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
global_socket.settimeout(timeout)
global_socket.bind((TCP_IP, TCP_PORT))
global_socket.listen()
global_conn = None
while True:
    try:
        global_conn, addr = global_socket.accept()
        break
    except Exception:
        print("BUSY WAITING FOR CONNECTION INITIALIZATION")
global_conn.settimeout(timeout)
time.sleep(0.5)

class ModalTimerOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"

    _timer = None

    def updateRocket(self, data):
        rocket = bpy.data.objects["Rocket"]
        vec = Vector((1.0, 0.0, 0.0))
        rocket.location = rocket.location + vec

    def modal(self, context, event):
        if event.type in {'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            try:
                data = global_conn.recv(BUFFER_SIZE)
                print("received data:", data)
                # no data left
                print(data.decode('utf-8'))
                if len(data.decode('utf-8')) == 1:
                    self.cancel(context)
                self.updateRocket(data)
            except Exception as e:
                print("Finish")
                print(e)
                self.cancel(context)
                return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        global_conn.close()
        global_socket.shutdown(0)
        global_socket.close()



def register():
    bpy.utils.register_class(ModalTimerOperator)


def unregister():
    bpy.utils.unregister_class(ModalTimerOperator)


if __name__ == "__main__":
    try:
        register()
        bpy.ops.wm.modal_timer_operator()
    except Exception as e:
        global_conn.close()
        global_socket.shutdown(0)
        global_socket.close()
