from dualsense_controller import DualSenseController
from time import sleep

device_infos = DualSenseController.enumerate_devices()
if len(device_infos) > 1:
    raise Exception('No Controller Connected')

is_running = True
controller = DualSenseController()

def stop():
    global is_running
    is_running = False

def on_left_changed(left_stick_y):
    print("LEFT: " + str(left_stick_y))

def on_error(error):
    print(f'an error occured: {error}')
    stop()

controller.left_stick_y.on_change(on_left_changed)

controller.on_error(on_error)
controller.activate()

while is_running:
    sleep(0.001)

controller.deactivate()
